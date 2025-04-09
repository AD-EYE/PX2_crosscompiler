// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ros/console.h"

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>
#include <dw/core/NvMedia.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// Image
#include <dw/image/ImageStreamer.h>

#include <dw/image/FormatConverter_vibrante.h>

// nvmedia for surface map
#include <nvmedia_2d.h>
#include "nvmedia_image.h"
#include "nvmedia_ijpe.h"
#include "nvmedia_surface.h"

// Renderer
#include <dw/renderer/Renderer.h>

#include <sstream>

#include <boost/program_options.hpp>

// OpenCV
#include <opencv2/opencv.hpp>


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
    ros::Publisher gmsl_pub_img_; // Removed gmsl_pub_ as it wasn't used
    // sensor_msgs::ImagePtr ros_img_ptr_; // Removed as it's created locally in publish loop


    // Image handles and properties
    dwImageHandle_t frame_rgb_ = DW_NULL_HANDLE;
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwImageProperties camera_image_properties_; // Not explicitly used, could be removed if not needed later
    dwCameraProperties camera_properties_;

    // Resolution properties - FIXED
    const int TARGET_WIDTH = 960;
    const int TARGET_HEIGHT = 604;

    po::variables_map args_;

public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // instantiate Driveworks SDK context
            dwContextParameters sdk_params = {};
            CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));

            // create HAL module of the SDK
            CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
        }

        //------------------------------------------------------------------------------
        // initializes camera
        // - the SensorCamera module
        // -----------------------------------------
        {
            dwSensorParams params;
            std::string parameter_string = std::string("output-format=yuv,fifo-size=3");

            parameter_string             += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string             += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string             += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

            if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                // it's a custom board, use the board specific extra configurations
                parameter_string             += ",custom-board=1";

                // pass an extra set of parameter in custom-config
                params.auxiliarydata           = args_["custom-config"].as<std::string>().c_str(); // Fixed typo: custom-board -> custom-config
            }

            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));

            // sensor can take some time to start, it's possible to call the read function and check if the return status is ok
            // before proceding
            CHECK_DW_ERROR(dwSensor_start(camera_));

            dwCameraFrameHandle_t frame;
            dwStatus status = DW_NOT_READY;
            dwTime_t start_time = 0; // Initialize
            CHECK_DW_ERROR(dwContext_getCurrentTime(&start_time, sdk_)); // Get start time correctly
            const dwTime_t MAX_WAIT_US = 10000000; // 10 seconds max wait
            do {
                status = dwSensorCamera_readFrame(&frame, 0, 66000, camera_);
                 if (status == DW_SUCCESS) {
                    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame)); // Return the frame immediately after successful read
                    break; // Exit loop if successful
                }

                // Check for timeout correctly
                dwTime_t current_time = 0;
                CHECK_DW_ERROR(dwContext_getCurrentTime(&current_time, sdk_));
                if (current_time - start_time > MAX_WAIT_US) {
                    // Construct the error message before throwing
                    std::string error_msg = "Timeout waiting for camera to start. Last status: " + std::string(dwGetStatusName(status));
                    throw std::runtime_error(error_msg);
                }
                 // Optional: Short sleep to avoid busy-waiting
                 // std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Requires #include <thread> and #include <chrono>
            } while (status == DW_NOT_READY || status == DW_TIME_OUT); // Check for timeout too

            // something wrong happened, aborting
            if (status != DW_SUCCESS) {
                 std::string error_msg = "Camera did not start correctly after loop. Final status: " + std::string(dwGetStatusName(status));
                throw std::runtime_error(error_msg);
            }

            // Frame already returned inside the loop if successful

            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Successfully initialized camera with native resolution of %dx%d at framerate of %f FPS. Publishing at fixed %dx%d.\n",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate,
                TARGET_WIDTH, TARGET_HEIGHT);
        }

         //ROS initialization
        {
            ros::VP_string ros_str; // This is usually empty for node init, handled by ROS internally
            // ros::init(ros_str, "camera_gmsl"); // Can cause issues if argc/argv not passed, simpler init below
            int ros_argc = 0; char** ros_argv = nullptr; // Fake argc/argv for init
            ros::init(ros_argc, ros_argv, "camera_gmsl_fixed_res"); // Use a slightly different name
            ros::NodeHandle n;
            gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);

            // ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>(); // Moved inside loop
            ROS_INFO("Successfully initialized ROS node: camera_gmsl_fixed_res\n" );
        }

        //Nvmedia initialization for RGBA image
        {
            dwImageProperties rgb_img_prop{};
            // Use camera native resolution here for the intermediate RGBA buffer
            rgb_img_prop.height = camera_properties_.resolution.y;
            rgb_img_prop.width = camera_properties_.resolution.x;
            rgb_img_prop.type = DW_IMAGE_NVMEDIA;
            rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            CHECK_DW_ERROR(dwImage_create(&frame_rgb_,  rgb_img_prop, sdk_)); // Corrected handle: sdk_
            ROS_INFO("Successfully initialized NvMedia RGBA image buffer (%dx%d).\n", rgb_img_prop.width, rgb_img_prop.height );
        }

    }

    ~CameraGMSL()
    {
        ROS_INFO("Shutting down CameraGMSL node...");
        if (camera_ != DW_NULL_HANDLE) { // Check if handle is valid before using
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_); // Sets camera_ to DW_NULL_HANDLE
        }

        // Destroy created image if it was created
        if (frame_rgb_ != DW_NULL_HANDLE) {
             dwImage_destroy(&frame_rgb_); // Sets frame_rgb_ to DW_NULL_HANDLE
        }

        if (sal_ != DW_NULL_HANDLE) {
            dwSAL_release(&sal_); // Sets sal_ to DW_NULL_HANDLE
        }
        if (sdk_ != DW_NULL_HANDLE) {
             dwRelease(&sdk_); // Sets sdk_ to DW_NULL_HANDLE
        }

        // dwLogger_release(); // Logger release is often handled internally or globally

        ROS_INFO("CameraGMSL resources released.");
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type - %s", cam_type.c_str());
        ROS_INFO("Starting to publish images at fixed resolution %dx%d", TARGET_WIDTH, TARGET_HEIGHT);

        try
        {
            ros::Rate loop_rate(15); // Consider matching camera framerate if possible/needed
            uint32_t count = 0;
            while (ros::ok())
            {
                dwTime_t timeout = 132000; // Corresponds roughly to 1/15fps + buffer, might need adjustment
                dwCameraFrameHandle_t frame = DW_NULL_HANDLE;
                uint32_t camera_sibling_id = 0; // Assuming single camera or master camera on port
                dwImageHandle_t frame_yuv = DW_NULL_HANDLE; // Initialize to null
                dwImageNvMedia* nvmedia_yuv_img_ptr = nullptr;
                dwImageNvMedia* nvmedia_rgb_img_ptr = nullptr;

                sensor_msgs::ImagePtr ros_img_ptr = boost::make_shared<sensor_msgs::Image>(); // Create a new message for each iteration
                std_msgs::Header header;
                header.seq = count;
                header.stamp = ros::Time::now();
                header.frame_id = "camera_link"; // Assign a frame_id

                dwStatus read_status;

                // Read from the camera
                read_status = dwSensorCamera_readFrame(&frame, camera_sibling_id, timeout, camera_);
                if (read_status == DW_TIME_OUT) {
                    ROS_WARN_THROTTLE(1.0, "Timeout reading camera frame."); // Warn periodically on timeout
                    continue; // Skip this iteration
                } else if (read_status != DW_SUCCESS) {
                    // Handle other potential errors
                    CHECK_DW_ERROR(read_status); // Throw exception for other errors
                }


                // Convert from YUV to RGB
                // Get the NvMedia YUV image handle from the camera frame
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));

                 // Check if the retrieved pointer is valid
                if (!nvmedia_yuv_img_ptr || !nvmedia_yuv_img_ptr->img) {
                    ROS_ERROR("Failed to get valid NvMedia YUV image from camera frame.");
                    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame)); // Must return the frame even on error
                    continue;
                }

                // Create a temporary DW image handle bound to the NvMedia YUV image
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));

                // Perform the color conversion YUV -> RGBA into our pre-allocated RGBA buffer
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));

                // Get the NvMedia handle for the resulting RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));

                 // Check if the retrieved pointer is valid
                if (!nvmedia_rgb_img_ptr || !nvmedia_rgb_img_ptr->img) {
                    ROS_ERROR("Failed to get valid NvMedia RGBA image after conversion.");
                    CHECK_DW_ERROR(dwImage_destroy(&frame_yuv)); // Clean up temporary YUV handle
                    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
                    continue;
                }

                // Lock the NvMedia RGBA image surface to access its data
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgb_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // Get original image dimensions (from the RGBA buffer, which matches camera native)
                    int original_height = nvmedia_rgb_img_ptr->prop.height;
                    int original_width = nvmedia_rgb_img_ptr->prop.width;

                    // Create an OpenCV Mat header pointing to the NvMedia buffer (RGBA format)
                    // WARNING: This data might not be CPU-accessible or might be tiled.
                    // Direct access might be slow or incorrect depending on the platform memory layout.
                    // Consider using DW->CPU conversion functions if performance is an issue or access fails.
                    cv::Mat original_image(original_height, original_width, CV_8UC4, surfaceMap.surface[0].mapping, surfaceMap.surface[0].pitch);

                    // Create a Mat for the target resolution
                    cv::Mat resized_image;

                    // Check if resize is actually needed (if native res matches target res)
                    if (original_width == TARGET_WIDTH && original_height == TARGET_HEIGHT) {
                        // If no resize needed, just reference the original data (if continuous)
                        // Or clone if we need a separate copy or it's not continuous
                        if (original_image.isContinuous()) {
                             resized_image = original_image; // Reference (zero-copy if possible)
                        } else {
                             original_image.copyTo(resized_image); // Copy if not continuous
                        }
                    } else {
                        // Perform resizing using OpenCV
                        cv::resize(original_image, resized_image, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_LINEAR);
                    }


                    // Manually convert the resized OpenCV Mat to a ROS message
                    ros_img_ptr->header = header; // Assign header
                    ros_img_ptr->height = TARGET_HEIGHT;
                    ros_img_ptr->width = TARGET_WIDTH;
                    ros_img_ptr->encoding = sensor_msgs::image_encodings::RGBA8; // Encoding matches our buffer
                    ros_img_ptr->is_bigendian = false;
                    ros_img_ptr->step = resized_image.step; // Use step from the (potentially resized) cv::Mat

                    // Copy the resized image data
                    size_t img_size = resized_image.total() * resized_image.elemSize(); // More robust size calculation
                    ros_img_ptr->data.resize(img_size);

                    // Copy the image data (handle non-continuous memory)
                    if(resized_image.isContinuous()) {
                        memcpy(&ros_img_ptr->data[0], resized_image.data, img_size);
                    } else {
                         // Copy row by row if memory is not continuous
                        for(int i = 0; i < TARGET_HEIGHT; ++i) {
                            memcpy(&ros_img_ptr->data[i * ros_img_ptr->step],
                                   resized_image.ptr<uchar>(i),
                                   ros_img_ptr->step); // Copy full row step bytes
                        }
                    }

                    // Publish the resized image
                    gmsl_pub_img_.publish(ros_img_ptr);

                    // Unlock the NvMedia image
                    NvMediaImageUnlock(nvmedia_rgb_img_ptr->img);
                }
                else {
                     ROS_ERROR("Failed to lock NvMedia RGBA image surface.");
                     // No need to unlock if lock failed
                }


                // Cleanup for this frame
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv)); // Destroy temporary YUV wrapper handle
                CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame)); // Return camera frame to the driver

                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            }
        }
        catch (const std::runtime_error &e) // Catch by const reference
        {
            ROS_ERROR("Runtime error in publish loop: %s", e.what());
            ros::shutdown(); // Shutdown ROS on critical error
        }
        catch (...) // Catch any other unexpected exceptions
        {
            ROS_ERROR("Unknown exception caught in publish loop!");
            ros::shutdown();
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, char *argv[]) // Use standard main signature
{
    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"),
            "Camera GMSL type (see sample_sensors_info for all available camera types on this platform)")
        ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [default a]\n"
                              "a - port AB on px2, A on ddpx\n"
                              "c - port CD on px2, C on ddpx\n"
                              "e - port EF on px2, E on ddpx\n"
                              "g - G on ddpx only")
        ("tegra-slave", po::value<std::string>()-> default_value("0"),
            "Optional parameter used only for Tegra B, enables slave mode.")
        ("custom-board", po::value<std::string>()-> default_value("0"), "If '1', then the configuration for board and camera "
                              "will be input from the config-file")
        ("custom-config", po::value<std::string>()-> default_value(""), "Set of custom board extra configuration (used if custom-board=1)");
        // Removed "resolution-ratio" option

    po::variables_map args;
    try {
        // Pass argc and argv to the parser
        po::store(po::parse_command_line(argc, argv, desc), args);

        if (args.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(args); // Important: This checks for required options and applies defaults

    } catch (const po::error &ex) {
        std::cerr << "Error parsing command line: " << ex.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }


    // Initialize ROS properly (needs argc, argv)
    // Note: ROS initialization is now inside CameraGMSL constructor,
    // but it's generally better practice to do it here in main before creating ROS-dependent objects.
    // Let's keep it in the constructor for now as per original structure, but be aware.
    // ros::init(argc, argv, "camera_gmsl_fixed_res_node"); // Example if done in main

    try {
        CameraGMSL cam(args);
        cam.publish();
    } catch (const std::runtime_error &e) {
         // Use ROS logging mechanisms if ROS is initialized, otherwise cerr
         if (ros::isInitialized()) {
             ROS_FATAL("Initialization failed: %s", e.what());
         } else {
             std::cerr << "Initialization failed: " << e.what() << std::endl;
         }
         return 1; // Indicate failure
    } catch (...) {
         if (ros::isInitialized()) {
             ROS_FATAL("Caught unknown exception during initialization!");
         } else {
            std::cerr << "Caught unknown exception during initialization!" << std::endl;
         }
         return 1;
    }


    return 0; // Indicate success
}
