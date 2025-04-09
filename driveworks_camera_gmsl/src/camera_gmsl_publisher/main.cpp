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
#include <dw/image/Image.h> // For dwImage_getProperties, dwImage_createAndBindNvMedia, dwImage_destroy

// nvmedia for surface map
#include <nvmedia_2d.h>
#include "nvmedia_image.h"
// #include "nvmedia_ijpe.h"
#include "nvmedia_surface.h"

// #include <dw/renderer/Renderer.h> // Not used

#include <sstream>
#include <string> // For std::to_string
#include <stdexcept> // For std::runtime_error
#include <memory> // **** ADDED for std::unique_ptr ****

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
            // ***** Request RGBA output directly *****
            std::string parameter_string = std::string("output-format=rgba,fifo-size=3");

            parameter_string             += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string             += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string             += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

             if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                parameter_string += ",custom-board=1";
                if (!args_["custom-config"].as<std::string>().empty()) {
                     params.auxiliarydata = args_["custom-config"].as<std::string>().c_str();
                } else if (args_["custom-board"].as<std::string>().compare("1") == 0) {
                    ROS_WARN("Custom board enabled ('custom-board=1') but 'custom-config' string is empty.");
                }
            }

            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            ROS_INFO("Creating sensor with parameters: %s", params.parameters);
            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
            ROS_INFO("Sensor created. Starting sensor...");
            CHECK_DW_ERROR(dwSensor_start(camera_));
            ROS_INFO("Sensor started. Waiting for first frame...");

            // Wait for camera to be ready
            dwCameraFrameHandle_t frame = DW_NULL_HANDLE; // Initialize properly
            dwStatus status = DW_NOT_READY;
            int retries = 10;
            while ((status == DW_NOT_READY || status == DW_TIME_OUT) && retries-- > 0) {
                 ros::Duration(0.2).sleep();
                 status = dwSensorCamera_readFrame(&frame, 0, 200000, camera_);
                 if (status == DW_SUCCESS) {
                     ROS_INFO("First frame read successfully during init.");
                     // === Debug: Check properties of the first frame's image ===
                     dwImageNvMedia* first_nvmedia_ptr = nullptr; // Initialize
                     dwStatus imgStatus = dwSensorCamera_getImageNvMedia(&first_nvmedia_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame);
                     if (imgStatus == DW_SUCCESS && first_nvmedia_ptr != nullptr && first_nvmedia_ptr->img != nullptr)
                     {
                         dwImageHandle_t temp_handle = DW_NULL_HANDLE; // Temporary handle for properties check
                         dwStatus bindStatus = dwImage_createAndBindNvMedia(&temp_handle, first_nvmedia_ptr->img);
                         if(bindStatus == DW_SUCCESS) {
                             dwImageProperties first_props;
                             dwStatus propStatus = dwImage_getProperties(&first_props, temp_handle);
                             if (propStatus == DW_SUCCESS) {
                                  ROS_INFO("Init Frame Properties: Type=%d, Format=%d, Width=%u, Height=%u",
                                            first_props.type, first_props.format, first_props.width, first_props.height);
                                  // Check if format is indeed RGBA (DW_IMAGE_FORMAT_RGBA_UINT8 = 3)
                                  if(first_props.format != DW_IMAGE_FORMAT_RGBA_UINT8) {
                                      ROS_WARN("Camera did not return RGBA format as requested! Actual format enum: %d", first_props.format);
                                  }
                             } else {
                                  ROS_WARN("Could not get properties from temporary DW Image handle in init: %s", dwGetStatusName(propStatus));
                             }
                             // Destroy the temporary handle
                             dwImage_destroy(&temp_handle); // temp_handle is now null or invalid
                         } else {
                             ROS_WARN("Could not create/bind temporary DW Image handle in init: %s", dwGetStatusName(bindStatus));
                         }

                     } else {
                         ROS_WARN("Could not get NvMedia Image from first frame in init (Status: %s, Ptr: %p)",
                                   dwGetStatusName(imgStatus), static_cast<void*>(first_nvmedia_ptr));
                     }
                     // =========================================================
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame)); // Return the frame
                 } else {
                     ROS_WARN("Camera not ready yet or timeout during init read (Status: %s, Retries left: %d)", dwGetStatusName(status), retries);
                 }
            }
            if (status != DW_SUCCESS) {
                 ROS_ERROR("Final camera status after init retries: %s", dwGetStatusName(status));
                 if (camera_) { // Check if sensor was created before trying to release
                    dwSAL_releaseSensor(&camera_);
                    camera_ = DW_NULL_HANDLE;
                 }
                 throw std::runtime_error("Camera did not start correctly or provide frames after multiple attempts.");
            }

            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Successfully initialized camera. Native resolution reported: %dx%d at framerate: %f FPS\n",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);
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
            dwStatus stop_status = dwSensor_stop(camera_);
            if (stop_status != DW_SUCCESS) {
                ROS_ERROR("Error stopping sensor: %s", dwGetStatusName(stop_status));
            }
             ROS_INFO("Releasing sensor...");
            dwStatus release_status = dwSAL_releaseSensor(&camera_); // releaseSensor takes pointer-to-pointer
             if (release_status != DW_SUCCESS) {
                ROS_ERROR("Error releasing sensor: %s", dwGetStatusName(release_status));
            }
            camera_ = DW_NULL_HANDLE; // Ensure handle is null after release attempt
            ROS_INFO("Sensor release process completed.");
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
        ROS_INFO("Resource release process completed.");
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
                dwStatus read_status = dwSensorCamera_readFrame(&frame_handle, 0, timeout, camera_);
                if (read_status == DW_TIME_OUT) {
                     ROS_WARN_THROTTLE(1.0, "Timeout reading camera frame %d.", count);
                     if(frame_handle) { // Return frame even on timeout if acquired somehow? Check DW docs. Usually NULL on timeout.
                         dwSensorCamera_returnFrame(&frame_handle);
                         frame_handle = DW_NULL_HANDLE;
                     }
                     continue; // Skip this iteration
                } else if (read_status == DW_NOT_READY) {
                     ROS_WARN_THROTTLE(1.0, "Camera reported DW_NOT_READY reading frame %d. Waiting...", count);
                     ros::Duration(0.05).sleep(); // Short sleep if not ready
                     if(frame_handle) { dwSensorCamera_returnFrame(&frame_handle); frame_handle = DW_NULL_HANDLE;}
                     continue;
                }
                // Check for other errors after handling timeout/not_ready
                CHECK_DW_ERROR(read_status);
                 if (!frame_handle) { // Should not happen if status is DW_SUCCESS, but good check
                    ROS_ERROR("[%d] Read frame returned DW_SUCCESS but frame handle is NULL!", count);
                    continue;
                }

                // 2. Get NvMedia handle for the NATIVE RGBA image buffer
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_rgba_native_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));
                 if (!nvmedia_rgba_native_ptr || !nvmedia_rgba_native_ptr->img) {
                     ROS_ERROR("[%d] getImageNvMedia returned success but pointer or image is NULL!", count);
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle)); // Return frame before continuing
                     continue;
                 }

                // 3. Create a temporary DW image handle *bound* to the camera's native RGBA NvMedia image
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_rgba_native, nvmedia_rgba_native_ptr->img));


                // 4. ***** Perform GPU-accelerated Resizing ONLY *****
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgba_resized_, frame_rgba_native, sdk_));

                // 5. Get NvMedia handle for the *final, resized* RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgba_resized_ptr, frame_rgba_resized_));
                 if (!nvmedia_rgba_resized_ptr || !nvmedia_rgba_resized_ptr->img) {
                     ROS_ERROR("[%d] getNvMedia (resized) returned success but pointer or image is NULL!", count);
                     CHECK_DW_ERROR(dwImage_destroy(&frame_rgba_native)); // Clean up bound handle
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle)); // Return frame
                     continue;
                 }

                // 6. Access the final NvMedia RGBA image data to copy to ROS message
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgba_resized_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // 7. Populate the ROS message
                    ros_img_ptr->header = header;
                    ros_img_ptr->height = TARGET_HEIGHT;
                    ros_img_ptr->width = TARGET_WIDTH;
                    ros_img_ptr->encoding = sensor_msgs::image_encodings::RGBA8;
                    ros_img_ptr->is_bigendian = false;
                    ros_img_ptr->step = static_cast<uint32_t>(TARGET_WIDTH * 4);

                    size_t img_size = static_cast<size_t>(ros_img_ptr->step) * TARGET_HEIGHT;
                    ros_img_ptr->data.resize(img_size);

                    // 8. Copy data from GPU (mapped NvMedia) to CPU (ROS message)
                    size_t expected_pitch = static_cast<size_t>(TARGET_WIDTH) * 4;
                    if (surfaceMap.surface[0].mapping == nullptr) {
                        ROS_ERROR("[%d] NvMediaImageLock succeeded but surface mapping is NULL!", count);
                        // Unlock before continuing/throwing
                        NvMediaImageUnlock(nvmedia_rgba_resized_ptr->img);
                    } else if (surfaceMap.surface[0].pitch == expected_pitch) {
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

                    // 9. Publish the ROS image message (only if data was valid)
                     if(surfaceMap.surface[0].mapping != nullptr) {
                        gmsl_pub_img_.publish(ros_img_ptr);
                     }

                    // 10. Unlock the NvMedia image surface (if locked)
                    // Check if mapping was NULL before trying to unlock might be needed depending on NvMedia behavior
                    NvMediaImageUnlock(nvmedia_rgba_resized_ptr->img);

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
        catch (const std::runtime_error &e)
        {
            ROS_ERROR("Runtime error in publish loop: %s", e.what());
        }
        catch (const std::exception &e)
        {
             ROS_ERROR("Standard exception caught in publish loop: %s", e.what());
        }
        catch (...)
        {
            ROS_ERROR("Unknown exception caught in publish loop.");
        }
         ROS_INFO("Publish loop finished.");
    }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_gmsl_gpu_resize_rgba");

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

    // Use unique_ptr for safer resource management (C++11 style)
    std::unique_ptr<CameraGMSL> cam_ptr;

    try {
        ROS_INFO("Creating CameraGMSL object (RGBA input, GPU resize)...");
        // **** MODIFICATION: Use C++11 style unique_ptr creation ****
        cam_ptr.reset(new CameraGMSL(args));
        ROS_INFO("CameraGMSL object created. Starting publishing...");
        cam_ptr->publish(); // Call publish method

    } catch (const std::runtime_error &e) {
        ROS_FATAL("Initialization or runtime error: %s", e.what());
        // cam_ptr will be automatically destroyed here if partially initialized
        return 1;
    } catch (const std::exception& e) {
        ROS_FATAL("An unexpected standard error occurred: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("An unknown error occurred.");
        return 1;
    }

    ROS_INFO("Camera GMSL node (RGBA input, GPU resize) shutting down normally.");
    return 0;
}
