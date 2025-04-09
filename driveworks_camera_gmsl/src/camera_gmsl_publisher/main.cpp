// ROS
#include "ros/ros.h"
#include "std_msgs/String.h" // Keep? Maybe not needed directly.
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ros/console.h"

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>
#include <dw/core/NvMedia.h> // Needed for dwImageNvMedia, NvMediaImageLock etc.

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// Image
#include <dw/image/ImageStreamer.h> // Keep? Maybe not needed directly.
#include <dw/image/FormatConverter.h> // dwImage_copyConvert is here

// nvmedia for surface map
#include <nvmedia_2d.h>
#include "nvmedia_image.h"
// #include "nvmedia_ijpe.h" // Likely not needed now
#include "nvmedia_surface.h"

// Renderer - Not used in this snippet
// #include <dw/renderer/Renderer.h>

#include <sstream>

#include <boost/program_options.hpp>

// OpenCV is no longer needed for resizing
// #include <opencv2/opencv.hpp>


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
    // dwImageProperties camera_image_properties_; // Not actively used
    dwCameraProperties camera_properties_;

    po::variables_map args_;

    // Fixed target resolution
    const int TARGET_WIDTH = 512;
    const int TARGET_HEIGHT = 512;

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
            std::string parameter_string = std::string("output-format=yuv,fifo-size=3"); // Camera outputs YUV

            parameter_string             += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string             += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string             += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();
            // ... (rest of parameter string logic) ...
             if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                parameter_string += ",custom-board=1";
                params.auxiliarydata = args_["custom-config"].as<std::string>().c_str(); // Corrected: use custom-config value
            }


            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
            CHECK_DW_ERROR(dwSensor_start(camera_));

            // Wait for camera to be ready
            dwCameraFrameHandle_t frame;
            dwStatus status = DW_NOT_READY;
            int retries = 5; // Add retries to avoid infinite loop
            while (status == DW_NOT_READY && retries-- > 0) {
                 ros::Duration(0.1).sleep(); // Short sleep
                 status = dwSensorCamera_readFrame(&frame, 0, 100000, camera_); // Increased timeout slightly
                 if (status == DW_SUCCESS) {
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
                 } else if (status != DW_NOT_READY) {
                     // Log unexpected errors during startup check
                     ROS_WARN("Error checking camera status during init: %s", dwGetStatusName(status));
                 }
            }
            if (status != DW_SUCCESS) {
                throw std::runtime_error("Camera did not start correctly after multiple attempts.");
            }


            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Successfully initialized camera with native resolution of %dx%d at framerate of %f FPS\n",
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
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
            camera_ = DW_NULL_HANDLE;
        }

        // Destroy the resized RGBA image handle
        if (frame_rgba_resized_) {
            dwImage_destroy(&frame_rgba_resized_);
            frame_rgba_resized_ = DW_NULL_HANDLE;
        }

        if (sal_) {
            dwSAL_release(&sal_);
            sal_ = DW_NULL_HANDLE;
        }
        if (sdk_) {
            dwRelease(&sdk_);
            sdk_ = DW_NULL_HANDLE;
        }
        ROS_INFO("Resources released.");
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type - %s", cam_type.c_str());
        ROS_INFO("Starting GPU-accelerated publishing with fixed resolution: %dx%d", TARGET_WIDTH, TARGET_HEIGHT);

        try
        {
            ros::Rate loop_rate(15); // Target publish rate
            int count = 0;
            while (ros::ok())
            {
                dwTime_t timeout = 132000; // Microseconds
                dwCameraFrameHandle_t frame_handle = DW_NULL_HANDLE;
                dwImageHandle_t frame_yuv_wrapper = DW_NULL_HANDLE; // Wrapper for the camera's NvMedia YUV
                dwImageNvMedia* nvmedia_yuv_img_ptr = nullptr;
                dwImageNvMedia* nvmedia_rgba_resized_ptr = nullptr;

                // Create a new ROS image message for each iteration
                sensor_msgs::ImagePtr ros_img_ptr = boost::make_shared<sensor_msgs::Image>();
                std_msgs::Header header;
                header.seq = count;
                header.stamp = ros::Time::now();
                header.frame_id = "camera_frame"; // Assign a frame_id

                // 1. Read frame from the camera (YUV format as configured)
                CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame_handle, 0, timeout, camera_));

                // 2. Get NvMedia handle for the native YUV image buffer
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));

                // 3. Create a temporary DW image handle *bound* to the camera's YUV NvMedia image
                //    This doesn't copy data, just creates a DW wrapper needed for dwImage_copyConvert.
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv_wrapper, nvmedia_yuv_img_ptr->img));

                // 4. ***** Perform GPU-accelerated Conversion AND Resizing *****
                //    Input: frame_yuv_wrapper (original resolution, YUV)
                //    Output: frame_rgba_resized_ (TARGET_WIDTHxTARGET_HEIGHT, RGBA)
                //    dwImage_copyConvert handles both steps using hardware acceleration.
                // ROS_DEBUG("Converting YUV to RGBA and resizing..."); // Use ROS_DEBUG
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgba_resized_, frame_yuv_wrapper, sdk_));
                // ROS_DEBUG("Conversion and resize complete.");

                // 5. Get NvMedia handle for the *final, resized* RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgba_resized_ptr, frame_rgba_resized_));

                // 6. Access the final NvMedia RGBA image data to copy to ROS message
                NvMediaImageSurfaceMap surfaceMap;
                // ROS_DEBUG("Locking final RGBA NvMedia surface for read...");
                if (NvMediaImageLock(nvmedia_rgba_resized_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // ROS_DEBUG("Surface locked.");
                    // 7. Populate the ROS message directly from the mapped NvMedia buffer
                    ros_img_ptr->header = header;
                    ros_img_ptr->height = TARGET_HEIGHT; // Use constants
                    ros_img_ptr->width = TARGET_WIDTH;  // Use constants
                    ros_img_ptr->encoding = sensor_msgs::image_encodings::RGBA8;
                    ros_img_ptr->is_bigendian = false;
                    ros_img_ptr->step = TARGET_WIDTH * 4; // RGBA = 4 bytes per pixel

                    // Allocate memory in the ROS message (CPU memory)
                    size_t img_size = ros_img_ptr->step * TARGET_HEIGHT;
                    ros_img_ptr->data.resize(img_size);

                    // 8. ***** Copy data from GPU (mapped NvMedia) to CPU (ROS message) *****
                    //    This is the main remaining data transfer.
                    // ROS_DEBUG("Copying final %dx%d RGBA data to ROS message...", TARGET_WIDTH, TARGET_HEIGHT);
                    // Ensure the pitch matches the width * bytes_per_pixel for a contiguous buffer
                    // NvMedia pitch might be different due to alignment. We need to copy row by row if pitch != width*4.
                    size_t expected_pitch = TARGET_WIDTH * 4;
                    if (surfaceMap.surface[0].pitch == expected_pitch) {
                        // If pitch matches, we can do a single memcpy (faster)
                         memcpy(ros_img_ptr->data.data(), surfaceMap.surface[0].mapping, img_size);
                    } else {
                        // If pitch is different (e.g., due to alignment), copy row by row
                        ROS_WARN_ONCE("NvMedia surface pitch (%zu) differs from expected (%zu). Copying row by row.",
                                      (size_t)surfaceMap.surface[0].pitch, expected_pitch);
                        uint8_t* ros_data_ptr = ros_img_ptr->data.data();
                        uint8_t* nv_data_ptr = static_cast<uint8_t*>(surfaceMap.surface[0].mapping);
                        for (uint32_t row = 0; row < TARGET_HEIGHT; ++row) {
                            memcpy(ros_data_ptr + row * expected_pitch, // Dest: ROS buffer row start
                                   nv_data_ptr + row * surfaceMap.surface[0].pitch, // Src: NvMedia row start
                                   expected_pitch); // Bytes to copy per row (TARGET_WIDTH * 4)
                        }
                    }
                    // ROS_DEBUG("Data copy complete.");

                    // 9. Publish the ROS image message
                    gmsl_pub_img_.publish(ros_img_ptr);
                    // ROS_DEBUG("Image published.");

                    // 10. Unlock the NvMedia image surface
                    NvMediaImageUnlock(nvmedia_rgba_resized_ptr->img);
                    // ROS_DEBUG("NvMedia surface unlocked.");
                } else {
                     ROS_WARN("Failed to lock NvMedia image for reading.");
                }

                // 11. Cleanup temporary YUV wrapper handle and return camera frame
                if (frame_yuv_wrapper) {
                    CHECK_DW_ERROR(dwImage_destroy(&frame_yuv_wrapper));
                    frame_yuv_wrapper = DW_NULL_HANDLE;
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
        catch (...) // Catch any other exceptions
        {
            ROS_ERROR("Unknown exception caught in publish loop.");
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_gmsl_gpu_resize"); // Updated node name

    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"),
            "Camera GMSL type")
        ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [a|c|e|g]")
        ("tegra-slave", po::value<std::string>()-> default_value("0"),
            "Optional: Tegra B slave mode [0|1]")
        ("custom-board", po::value<std::string>()-> default_value("0"), "Use custom board config [0|1]")
        ("custom-config", po::value<std::string>()-> default_value(""), "Path to custom board config file");

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

    try {
        ROS_INFO("Creating CameraGMSL object (GPU resize)...");
        CameraGMSL cam(args);
        ROS_INFO("CameraGMSL object created. Starting publishing...");
        cam.publish();

    } catch (const std::runtime_error &e) {
        ROS_FATAL("Initialization failed: %s", e.what());
        return 1;
    } catch (const std::exception& e) {
        ROS_FATAL("An unexpected error occurred during initialization: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("An unknown error occurred during initialization.");
        return 1;
    }

    ROS_INFO("Camera GMSL node (GPU resize) shutting down.");
    return 0;
}
