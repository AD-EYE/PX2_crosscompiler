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
    ros::Publisher gmsl_pub_img_; // Changed variable name for clarity
    ros::NodeHandle nh_;
    // Removed ros_img_ptr_ member variable, create locally in loop


    // Image handles and properties
    dwImageHandle_t frame_rgb_ = DW_NULL_HANDLE;
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwImageProperties camera_image_properties_; // Consider removing if not used elsewhere
    dwCameraProperties camera_properties_;

    po::variables_map args_;

    // Fixed target resolution
    const int TARGET_WIDTH = 512;
    const int TARGET_HEIGHT = 512;

public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        // Create ROS NodeHandle - ros::init is already called inside main
        gmsl_pub_img_ = nh_.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);
        // Removed ros_img_ptr_ initialization here
        ROS_INFO("Successfully initialized ROS publisher\n");

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
                params.auxiliarydata           = args_["custom-board"].as<std::string>().c_str();
            }

            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));

            // sensor can take some time to start, it's possible to call the read function and check if the return status is ok
            // before proceding
            CHECK_DW_ERROR(dwSensor_start(camera_));

            dwCameraFrameHandle_t frame;
            dwStatus status = DW_NOT_READY;
            do {
                status = dwSensorCamera_readFrame(&frame, 0, 66000, camera_);
            } while (status == DW_NOT_READY);

            // something wrong happened, aborting
            if (status != DW_SUCCESS) {
                throw std::runtime_error("Cameras did not start correctly");
            }

            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));

            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Successfully initialized camera with native resolution of %dx%d at framerate of %f FPS\n",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);
        }

        //Nvmedia initialization for the RGB image that will hold the converted frame
        {
            dwImageProperties rgb_img_prop{};
            // Set properties based on the original camera resolution, conversion happens before resizing
            rgb_img_prop.height = camera_properties_.resolution.y;
            rgb_img_prop.width = camera_properties_.resolution.x;
            rgb_img_prop.type = DW_IMAGE_NVMEDIA;
            rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8; // Conversion target format
            CHECK_DW_ERROR(dwImage_create(&frame_rgb_, rgb_img_prop, sdk_));
            ROS_INFO("Successfully initialized intermediate NvMedia RGBA image handle.\n");
        }
    }

    ~CameraGMSL()
    {
        ROS_INFO("Destructor called. Releasing resources.");
        if (camera_) {
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
            camera_ = DW_NULL_HANDLE; // Good practice to null handles after release
        }

        //destroy created image
        if (frame_rgb_) {
            dwImage_destroy(&frame_rgb_);
            frame_rgb_ = DW_NULL_HANDLE; // Good practice
        }

        if (sal_) {
            dwSAL_release(&sal_);
            sal_ = DW_NULL_HANDLE; // Good practice
        }
        if (sdk_) {
            dwRelease(&sdk_);
            sdk_ = DW_NULL_HANDLE; // Good practice
        }
        // dwLogger_release(); // Often called automatically or managed differently
        ROS_INFO("Resources released.");
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type - %s", cam_type.c_str());
        ROS_INFO("Starting to publish images with fixed resolution: %dx%d", TARGET_WIDTH, TARGET_HEIGHT);

        // Removed the dynamic resolution ratio logic

        try
        {
            ros::Rate loop_rate(15); // Target publish rate
            int count = 0;
            while (ros::ok())
            {
                dwTime_t timeout = 132000; // Microseconds, slightly more than 2x frame time at 15fps
                dwCameraFrameHandle_t frame_handle = DW_NULL_HANDLE; // Use a different name than member variable
                dwImageHandle_t frame_yuv = DW_NULL_HANDLE;
                dwImageNvMedia* nvmedia_yuv_img_ptr = nullptr;
                dwImageNvMedia* nvmedia_rgb_img_ptr = nullptr;


                // Create a new ROS image message for each iteration
                sensor_msgs::ImagePtr ros_img_ptr = boost::make_shared<sensor_msgs::Image>();
                std_msgs::Header header;
                header.seq = count;
                header.stamp = ros::Time::now();
                header.frame_id = "camera_frame"; // Assign a frame_id

                // 1. Read frame from the camera
                // ROS_DEBUG("Attempting to read frame..."); // Use ROS_DEBUG for frequent messages
                CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame_handle, 0 /*sibling_id*/, timeout, camera_));
                // ROS_DEBUG("Frame read successfully.");

                // 2. Get NvMedia handle for the native YUV image
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));

                // 3. Create a temporary DW image handle bound to the YUV NvMedia image
                //    This is needed for the dwImage_copyConvert function
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));

                // 4. Convert YUV image to the pre-allocated RGBA image (frame_rgb_)
                // ROS_DEBUG("Converting YUV to RGBA...");
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));
                // ROS_DEBUG("Conversion complete.");

                // 5. Get NvMedia handle for the converted RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));

                // 6. Access NvMedia RGBA image data for OpenCV processing
                NvMediaImageSurfaceMap surfaceMap;
                // ROS_DEBUG("Locking NvMedia surface for read...");
                if (NvMediaImageLock(nvmedia_rgb_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // ROS_DEBUG("Surface locked.");
                    // Get original image dimensions from the NvMedia properties
                    int original_height = nvmedia_rgb_img_ptr->prop.height;
                    int original_width = nvmedia_rgb_img_ptr->prop.width;

                    // Create an OpenCV Mat **referencing** the NvMedia buffer (RGBA format). No data copy here.
                    cv::Mat original_image_rgba(original_height, original_width, CV_8UC4, surfaceMap.surface[0].mapping, surfaceMap.surface[0].pitch); // Use pitch!

                    // Create a Mat for the resized image (this will allocate memory)
                    cv::Mat resized_image;

                    // Resize the image using OpenCV to the fixed target size
                    // ROS_DEBUG("Resizing image...");
                    cv::resize(original_image_rgba, resized_image, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_LINEAR);
                    // ROS_DEBUG("Resizing complete.");

                    // 7. Manually convert the *resized* OpenCV Mat to a ROS message
                    ros_img_ptr->header = header;
                    ros_img_ptr->height = TARGET_HEIGHT;
                    ros_img_ptr->width = TARGET_WIDTH;
                    ros_img_ptr->encoding = sensor_msgs::image_encodings::RGBA8; // Encoding is RGBA
                    ros_img_ptr->is_bigendian = false;
                    ros_img_ptr->step = TARGET_WIDTH * 4; // 4 bytes per pixel (R, G, B, A)

                    // Allocate memory in the ROS message
                    size_t img_size = ros_img_ptr->step * TARGET_HEIGHT;
                    ros_img_ptr->data.resize(img_size);

                    // Copy the resized image data into the ROS message buffer
                    // Check if the resized Mat is continuous for potentially faster copy
                    if(resized_image.isContinuous()) {
                        // ROS_DEBUG("Copying continuous data to ROS message...");
                        memcpy(ros_img_ptr->data.data(), resized_image.data, img_size);
                    } else {
                        // ROS_DEBUG("Copying non-continuous data to ROS message row by row...");
                        // If not continuous (e.g., ROI), copy row by row
                        uint8_t* ros_data_ptr = ros_img_ptr->data.data();
                        for(int i = 0; i < TARGET_HEIGHT; ++i) {
                            memcpy(ros_data_ptr + i * ros_img_ptr->step, // Destination in ROS msg
                                   resized_image.ptr<uchar>(i),        // Source row in OpenCV Mat
                                   ros_img_ptr->step);                 // Bytes per row
                        }
                    }
                    // ROS_DEBUG("Data copy complete.");

                    // 8. Publish the ROS image message
                    gmsl_pub_img_.publish(ros_img_ptr);
                    // ROS_DEBUG("Image published.");

                    // 9. Unlock the NvMedia image surface
                    NvMediaImageUnlock(nvmedia_rgb_img_ptr->img);
                    // ROS_DEBUG("NvMedia surface unlocked.");
                } else {
                     ROS_WARN("Failed to lock NvMedia image for reading.");
                }

                // 10. Cleanup temporary DW image handle and return camera frame
                if (frame_yuv) {
                    CHECK_DW_ERROR(dwImage_destroy(&frame_yuv)); // Destroy the temporary YUV wrapper
                    frame_yuv = DW_NULL_HANDLE;
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
        catch (const std::runtime_error &e) // Catch specific exception
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
    // First, initialize ROS
    ros::init(argc, argv, "camera_gmsl_fixed_res"); // Changed node name slightly

    // Define command line options using Boost Program Options
    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"),
            "Camera GMSL type (see sample_sensors_info for available types)")
        ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [a|c|e|g]")
        ("tegra-slave", po::value<std::string>()-> default_value("0"),
            "Optional: Tegra B slave mode [0|1]")
        ("custom-board", po::value<std::string>()-> default_value("0"), "Use custom board config [0|1]")
        ("custom-config", po::value<std::string>()-> default_value(""), "Path to custom board config file");
        // Removed "resolution-ratio" option

    po::variables_map args;

    try {
        // Parse command line arguments (need const_cast for older Boost versions with char**)
        po::store(po::parse_command_line(argc, const_cast<const char**>(argv), desc), args);

        if (args.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(args); // Check for errors and apply defaults

    } catch (const po::error &ex) {
        ROS_ERROR("Error parsing command line options: %s", ex.what());
        std::cerr << desc << std::endl;
        return 1;
    } catch (const std::exception &ex) {
        ROS_ERROR("Error initializing options: %s", ex.what());
        return 1;
    }


    try {
        ROS_INFO("Creating CameraGMSL object...");
        CameraGMSL cam(args);
        ROS_INFO("CameraGMSL object created. Starting publishing...");
        cam.publish();

    } catch (const std::runtime_error &e) {
        ROS_FATAL("Initialization failed: %s", e.what()); // Use ROS_FATAL for critical init errors
        return 1;
    } catch (const std::exception& e) {
        ROS_FATAL("An unexpected error occurred during initialization: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("An unknown error occurred during initialization.");
        return 1;
    }

    ROS_INFO("Camera GMSL node shutting down.");
    return 0;
}
