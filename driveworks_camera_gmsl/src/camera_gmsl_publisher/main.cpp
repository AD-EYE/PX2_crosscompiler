// ROS
#include "ros/ros.h"
#include "std_msgs/String.h" // Included but not directly used in logic
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ros/console.h"

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>
#include <dw/core/NvMedia.h> // Needed for dwImageNvMedia, NvMediaImageLock etc.
#include <dw/core/Status.h>   // For dwGetStatusName
#include <dw/core/Types.h>    // For dwImageFormat enum

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// Image
// #include <dw/image/ImageStreamer.h> // Not strictly needed here
#include <dw/image/FormatConverter.h> // dwImage_copyConvert is here
#include <dw/image/Image.h> // For dwImage_getProperties

// nvmedia for surface map
#include <nvmedia_2d.h>
#include "nvmedia_image.h"
// #include "nvmedia_ijpe.h"
#include "nvmedia_surface.h"

// #include <dw/renderer/Renderer.h> // Not used

#include <sstream>
#include <string> // For std::to_string
#include <stdexcept> // For std::runtime_error

#include <boost/program_options.hpp>

// OpenCV is NOT needed


#define CHECK_DW_ERROR(x) { \
                    dwStatus result = x; \
                    if(result!=DW_SUCCESS) { \
                        throw std::runtime_error(std::string("DW Error ") \
                                                + dwGetStatusName(result) \
                                                + std::string(" executing DW function:\n " #x) \
                                                + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
                    }};

namespace po = boost::program_options;

class CameraGMSL
{
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t sdk_                  = DW_NULL_HANDLE;
    dwSALHandle_t sal_                      = DW_NULL_HANDLE;

    // ROS variables
    ros::Publisher gmsl_pub_img_;
    ros::NodeHandle nh_;

    // Image handles and properties
    dwImageHandle_t frame_rgba_resized_ = DW_NULL_HANDLE; // Handle for the FINAL resized RGBA image on GPU
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwCameraProperties camera_properties_;

    po::variables_map args_;

    // Fixed target resolution (use unsigned type)
    const uint32_t TARGET_WIDTH = 512;
    const uint32_t TARGET_HEIGHT = 512;

public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        // Create ROS NodeHandle
        gmsl_pub_img_ = nh_.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);
        ROS_INFO("Successfully initialized ROS publisher\n");

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            dwContextParameters sdk_params = {};
            CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));
            CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
        }

        //------------------------------------------------------------------------------
        // Initialize camera
        //------------------------------------------------------------------------------
        {
            dwSensorParams params;
            // ***** MODIFICATION: Request RGBA output directly *****
            std::string parameter_string = std::string("output-format=rgba,fifo-size=3");

            parameter_string             += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string             += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string             += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

             if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                parameter_string += ",custom-board=1";
                // Ensure the custom-config parameter is actually used if needed by the board config
                if (!args_["custom-config"].as<std::string>().empty()) {
                     params.auxiliarydata = args_["custom-config"].as<std::string>().c_str();
                } else if (args_["custom-board"].as<std::string>().compare("1") == 0) {
                    // Warn if custom board is set but no config string is provided, might be needed
                    ROS_WARN("Custom board enabled ('custom-board=1') but 'custom-config' string is empty.");
                }
            }


            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            ROS_INFO("Creating sensor with parameters: %s", params.parameters); // Log parameters
            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
            ROS_INFO("Sensor created. Starting sensor...");
            CHECK_DW_ERROR(dwSensor_start(camera_));
            ROS_INFO("Sensor started. Waiting for first frame...");

            // Wait for camera to be ready
            dwCameraFrameHandle_t frame;
            dwStatus status = DW_NOT_READY;
            int retries = 10; // Increased retries slightly
            while ((status == DW_NOT_READY || status == DW_TIME_OUT) && retries-- > 0) {
                 ros::Duration(0.2).sleep(); // Slightly longer sleep
                 status = dwSensorCamera_readFrame(&frame, 0, 200000, camera_); // Increased timeout
                 if (status == DW_SUCCESS) {
                     ROS_INFO("First frame read successfully during init.");
                     // === Debug: Check properties of the first frame's image ===
                     dwImageHandle_t first_img_handle;
                     dwImageNvMedia* first_nvmedia_ptr;
                     dwStatus imgStatus = dwSensorCamera_getImageNvMedia(&first_nvmedia_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame);
                     if (imgStatus == DW_SUCCESS) {
                         dwImageProperties first_props;
                         imgStatus = dwImage_getPropertiesFromNvMedia(&first_props, first_nvmedia_ptr->img);
                         if (imgStatus == DW_SUCCESS) {
                              ROS_INFO("Init Frame Properties: Type=%d, Format=%d, Width=%u, Height=%u",
                                        first_props.type, first_props.format, first_props.width, first_props.height);
                              // Check if format is indeed RGBA (DW_IMAGE_FORMAT_RGBA_UINT8 = 3)
                              if(first_props.format != DW_IMAGE_FORMAT_RGBA_UINT8) {
                                  ROS_WARN("Camera did not return RGBA format as requested! Actual format enum: %d", first_props.format);
                              }
                         } else {
                              ROS_WARN("Could not get properties from NvMedia Image in init: %s", dwGetStatusName(imgStatus));
                         }
                     } else {
                         ROS_WARN("Could not get NvMedia Image from first frame in init: %s", dwGetStatusName(imgStatus));
                     }
                     // =========================================================
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame)); // Return the frame
                 } else {
                     ROS_WARN("Camera not ready yet or timeout during init read (Status: %s, Retries left: %d)", dwGetStatusName(status), retries);
                 }
            }
            if (status != DW_SUCCESS) {
                 ROS_ERROR("Final camera status after init retries: %s", dwGetStatusName(status));
                 dwSAL_releaseSensor(&camera_); // Clean up sensor if init failed
                 camera_ = DW_NULL_HANDLE;
                 throw std::runtime_error("Camera did not start correctly or provide frames after multiple attempts.");
            }


            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Successfully initialized camera. Native resolution reported: %dx%d at framerate: %f FPS\n",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);
             // Sanity check if reported resolution matches what we got in the first frame (if debug worked)
             // Note: camera_properties_ might reflect sensor capability, frame properties reflect actual output format resolution.
        }

        //------------------------------------------------------------------------------
        // Initialize the *final* RGBA image handle with TARGET resolution
        //------------------------------------------------------------------------------
        {
            dwImageProperties rgba_resized_prop{};
            rgba_resized_prop.height = TARGET_HEIGHT; // Target height
            rgba_resized_prop.width = TARGET_WIDTH;   // Target width
            rgba_resized_prop.type = DW_IMAGE_NVMEDIA; // Reside in GPU accessible memory
            rgba_resized_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8; // Target format
            CHECK_DW_ERROR(dwImage_create(&frame_rgba_resized_, rgba_resized_prop, sdk_));
            ROS_INFO("Successfully initialized NvMedia RGBA image handle for target resolution %dx%d.\n", TARGET_WIDTH, TARGET_HEIGHT);
        }
    }

    ~CameraGMSL()
    {
        ROS_INFO("Destructor called. Releasing resources.");
        if (camera_) {
            ROS_INFO("Stopping sensor...");
            dwSensor_stop(camera_);
             ROS_INFO("Releasing sensor...");
            dwSAL_releaseSensor(&camera_);
            camera_ = DW_NULL_HANDLE;
            ROS_INFO("Sensor released.");
        }

        // Destroy the resized RGBA image handle
        if (frame_rgba_resized_) {
             ROS_INFO("Destroying resized RGBA image handle...");
            dwImage_destroy(&frame_rgba_resized_);
            frame_rgba_resized_ = DW_NULL_HANDLE;
            ROS_INFO("Image handle destroyed.");
        }

        if (sal_) {
             ROS_INFO("Releasing SAL...");
            dwSAL_release(&sal_);
            sal_ = DW_NULL_HANDLE;
            ROS_INFO("SAL released.");
        }
        if (sdk_) {
            ROS_INFO("Releasing SDK context...");
            dwRelease(&sdk_);
            sdk_ = DW_NULL_HANDLE;
             ROS_INFO("SDK context released.");
        }
        ROS_INFO("Resources released.");
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type - %s", cam_type.c_str());
        ROS_INFO("Starting GPU-accelerated publishing (RGBA input, resize only) with fixed resolution: %dx%d", TARGET_WIDTH, TARGET_HEIGHT);

        try
        {
            ros::Rate loop_rate(15); // Target publish rate
            int count = 0;
            while (ros::ok())
            {
                dwTime_t timeout = 132000; // Microseconds
                dwCameraFrameHandle_t frame_handle = DW_NULL_HANDLE;

                // Handles for Native RGBA image from camera
                dwImageHandle_t frame_rgba_native = DW_NULL_HANDLE;
                dwImageNvMedia* nvmedia_rgba_native_ptr = nullptr;

                // Pointer for the final resized image's NvMedia struct
                dwImageNvMedia* nvmedia_rgba_resized_ptr = nullptr;

                // Create a new ROS image message for each iteration
                sensor_msgs::ImagePtr ros_img_ptr = boost::make_shared<sensor_msgs::Image>();
                std_msgs::Header header;
                header.seq = count;
                header.stamp = ros::Time::now();
                header.frame_id = "camera_frame"; // Assign a frame_id

                // 1. Read frame from the camera (expecting RGBA format)
                // ROS_DEBUG("Reading frame %d...", count); // Use ROS_DEBUG
                dwStatus read_status = dwSensorCamera_readFrame(&frame_handle, 0, timeout, camera_);
                if (read_status == DW_TIME_OUT) {
                     ROS_WARN_THROTTLE(1.0, "Timeout reading camera frame %d.", count); // Warn occasionally on timeout
                     continue; // Skip this iteration
                }
                CHECK_DW_ERROR(read_status); // Check for other errors
                // ROS_DEBUG("Frame %d read.", count);

                // 2. Get NvMedia handle for the NATIVE RGBA image buffer
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_rgba_native_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));

                // 3. Create a temporary DW image handle *bound* to the camera's native RGBA NvMedia image
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_rgba_native, nvmedia_rgba_native_ptr->img));

                 // === Optional Debug: Check native frame properties every N frames ===
                 /*
                 if (count % 100 == 0) { // Check every 100 frames
                     dwImageProperties native_props;
                     dwStatus prop_status = dwImage_getProperties(&native_props, frame_rgba_native);
                     if (prop_status == DW_SUCCESS) {
                         if(native_props.format != DW_IMAGE_FORMAT_RGBA_UINT8) {
                             ROS_WARN_THROTTLE(10.0,"[%d] Native frame format is NOT RGBA! Actual: %d", count, native_props.format);
                         }
                         if(native_props.width != camera_properties_.resolution.x || native_props.height != camera_properties_.resolution.y) {
                              ROS_WARN_THROTTLE(10.0,"[%d] Native frame size %ux%u differs from sensor properties %ux%u", count,
                                                native_props.width, native_props.height,
                                                camera_properties_.resolution.x, camera_properties_.resolution.y);
                         }
                     }
                 }
                 */
                 // =====================================================================


                // 4. ***** Perform GPU-accelerated Resizing ONLY *****
                //    Input: frame_rgba_native (original resolution, RGBA)
                //    Output: frame_rgba_resized_ (TARGET_WIDTHxTARGET_HEIGHT, RGBA)
                // ROS_DEBUG("Resizing frame %d...", count);
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgba_resized_, frame_rgba_native, sdk_)); // Should only resize now
                // ROS_DEBUG("Frame %d resized.", count);

                // 5. Get NvMedia handle for the *final, resized* RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgba_resized_ptr, frame_rgba_resized_));

                // 6. Access the final NvMedia RGBA image data to copy to ROS message
                NvMediaImageSurfaceMap surfaceMap;
                // ROS_DEBUG("Locking resized surface for frame %d...", count);
                if (NvMediaImageLock(nvmedia_rgba_resized_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // ROS_DEBUG("Surface locked for frame %d.", count);
                    // 7. Populate the ROS message directly from the mapped NvMedia buffer
                    ros_img_ptr->header = header;
                    ros_img_ptr->height = TARGET_HEIGHT;
                    ros_img_ptr->width = TARGET_WIDTH;
                    ros_img_ptr->encoding = sensor_msgs::image_encodings::RGBA8;
                    ros_img_ptr->is_bigendian = false;
                    ros_img_ptr->step = static_cast<uint32_t>(TARGET_WIDTH * 4);

                    // Allocate memory in the ROS message (CPU memory)
                    size_t img_size = static_cast<size_t>(ros_img_ptr->step) * TARGET_HEIGHT;
                    ros_img_ptr->data.resize(img_size);

                    // 8. ***** Copy data from GPU (mapped NvMedia) to CPU (ROS message) *****
                    // ROS_DEBUG("Copying data to ROS message for frame %d...", count);
                    size_t expected_pitch = static_cast<size_t>(TARGET_WIDTH) * 4;
                    if (surfaceMap.surface[0].pitch == expected_pitch) {
                         memcpy(ros_img_ptr->data.data(), surfaceMap.surface[0].mapping, img_size);
                    } else {
                        ROS_WARN_ONCE("NvMedia surface pitch (%zu) differs from expected (%zu). Copying row by row.",
                                      (size_t)surfaceMap.surface[0].pitch, expected_pitch);
                        uint8_t* ros_data_ptr = ros_img_ptr->data.data();
                        uint8_t* nv_data_ptr = static_cast<uint8_t*>(surfaceMap.surface[0].mapping);
                        for (uint32_t row = 0; row < TARGET_HEIGHT; ++row) {
                            memcpy(ros_data_ptr + row * expected_pitch,
                                   nv_data_ptr + row * surfaceMap.surface[0].pitch,
                                   expected_pitch);
                        }
                    }
                    // ROS_DEBUG("Data copy complete for frame %d.", count);

                    // 9. Publish the ROS image message
                    gmsl_pub_img_.publish(ros_img_ptr);
                    // ROS_DEBUG("Frame %d published.", count);

                    // 10. Unlock the NvMedia image surface
                    NvMediaImageUnlock(nvmedia_rgba_resized_ptr->img);
                    // ROS_DEBUG("Surface unlocked for frame %d.", count);
                } else {
                     ROS_WARN("Failed to lock NvMedia image for reading for frame %d.", count);
                }

                // 11. Cleanup temporary native RGBA wrapper handle and return camera frame
                if (frame_rgba_native) {
                    CHECK_DW_ERROR(dwImage_destroy(&frame_rgba_native));
                    frame_rgba_native = DW_NULL_HANDLE;
                }
                if (frame_handle) {
                    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));
                    frame_handle = DW_NULL_HANDLE;
                }

                // ROS spin and sleep
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            }
        }
        catch (const std::runtime_error &e) // Catch specific DW runtime errors
        {
            // Log the specific error from CHECK_DW_ERROR macro
            ROS_ERROR("Runtime error in publish loop: %s", e.what());
        }
        catch (const std::exception &e) // Catch other standard exceptions
        {
             ROS_ERROR("Standard exception caught in publish loop: %s", e.what());
        }
        catch (...) // Catch any other unknown exceptions
        {
            ROS_ERROR("Unknown exception caught in publish loop.");
        }
        // Destructor will handle resource cleanup when loop exits or exception occurs
         ROS_INFO("Publish loop finished.");
    }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_gmsl_gpu_resize_rgba"); // Updated node name

    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"),
            "Camera GMSL type")
        ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [a|c|e|g]")
        ("tegra-slave", po::value<std::string>()-> default_value("0"),
            "Optional: Tegra B slave mode [0|1]")
        ("custom-board", po::value<std::string>()-> default_value("0"), "Use custom board config [0|1]")
        ("custom-config", po::value<std::string>()-> default_value(""), "Path or string for custom board config");

    po::variables_map args;

    try {
        po::store(po::parse_command_line(argc, const_cast<const char**>(argv), desc), args);

        if (args.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        po::notify(args);

    } catch (const po::error &ex) {
        ROS_ERROR("Error parsing command line options: %s", ex.what());
        std::cerr << desc << std::endl;
        return 1;
    } catch (const std::exception &ex) {
        ROS_ERROR("Error initializing options: %s", ex.what());
        return 1;
    }

    std::unique_ptr<CameraGMSL> cam_ptr; // Use unique_ptr for safer resource management

    try {
        ROS_INFO("Creating CameraGMSL object (RGBA input, GPU resize)...");
        cam_ptr = std::make_unique<CameraGMSL>(args); // Create object
        ROS_INFO("CameraGMSL object created. Starting publishing...");
        cam_ptr->publish(); // Call publish method

    } catch (const std::runtime_error &e) {
        ROS_FATAL("Initialization or runtime error: %s", e.what()); // Use ROS_FATAL for critical errors
        // cam_ptr will be automatically destroyed here if partially initialized
        return 1;
    } catch (const std::exception& e) {
        ROS_FATAL("An unexpected standard error occurred: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("An unknown error occurred.");
        return 1;
    }

    // Destructor of CameraGMSL (via unique_ptr) will be called here upon normal exit
    ROS_INFO("Camera GMSL node (RGBA input, GPU resize) shutting down normally.");
    return 0;
}
