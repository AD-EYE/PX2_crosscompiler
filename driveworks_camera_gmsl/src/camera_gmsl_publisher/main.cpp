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
#include <dw/core/Status.h>
#include <dw/core/Types.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// Image
#include <dw/image/FormatConverter.h>
#include <dw/image/Image.h>

// nvmedia for surface map
#include <nvmedia_2d.h>
#include "nvmedia_image.h"
#include "nvmedia_surface.h"

#include <sstream>
#include <string>
#include <stdexcept>
#include <memory>

#include <boost/program_options.hpp>


#define CHECK_DW_ERROR(x) { \
                    dwStatus result = x; \
                    if(result!=DW_SUCCESS) { \
                        throw std::runtime_error(std::string("DW Error ") \
                                                + dwGetStatusName(result) \
                                                + std::string(" executing DW function:\n " #x) \
                                                + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
                    }};

// Helper function to get format string (basic implementation)
const char* dwImageFormatToStr(dwImageFormat format) {
    switch (format) {
        case DW_IMAGE_FORMAT_RGBA_UINT8: return "RGBA_UINT8";
        case DW_IMAGE_FORMAT_RGB_UINT8_PLANAR: return "RGB_UINT8_PLANAR";
        // Add common YUV formats - consult dw/core/Types.h for your DW version
        case DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR: return "YUV420_UINT8_PLANAR";
        case DW_IMAGE_FORMAT_YUV420_UINT8_SEMIPLANAR: return "YUV420_UINT8_SEMIPLANAR"; // Often NV12
        case DW_IMAGE_FORMAT_YUV422_UINT8_PLANAR: return "YUV422_UINT8_PLANAR";
        case DW_IMAGE_FORMAT_YUV422_UINT8_SEMIPLANAR: return "YUV422_UINT8_SEMIPLANAR"; // Often NV16
        case DW_IMAGE_FORMAT_YUV444_UINT8_PLANAR: return "YUV444_UINT8_PLANAR";
        case DW_IMAGE_FORMAT_YUV_UINT8_PLANAR: return "YUV_UINT8_PLANAR"; // Generic YUV Planar?
        // Add other formats as needed
        default: return "UNKNOWN_FORMAT";
    }
}


namespace po = boost::program_options;

class CameraGMSL
{
private:
    dwContextHandle_t sdk_ = DW_NULL_HANDLE;
    dwSALHandle_t sal_ = DW_NULL_HANDLE;
    ros::Publisher gmsl_pub_img_;
    ros::NodeHandle nh_;
    dwImageHandle_t frame_rgba_resized_ = DW_NULL_HANDLE;
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwCameraProperties camera_properties_;
    po::variables_map args_;
    const uint32_t TARGET_WIDTH = 512;
    const uint32_t TARGET_HEIGHT = 512;

public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        gmsl_pub_img_ = nh_.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);
        ROS_INFO("Successfully initialized ROS publisher\n");

        dwContextParameters sdk_params = {};
        CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));
        CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));

        dwSensorParams params;
        // ***** MODIFICATION: Request YUV output again *****
        std::string parameter_string = std::string("output-format=yuv,fifo-size=3");

        parameter_string += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
        parameter_string += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
        parameter_string += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

         if (args_["custom-board"].as<std::string>().compare("1") == 0) {
            parameter_string += ",custom-board=1";
            if (!args_["custom-config"].as<std::string>().empty()) {
                 params.auxiliarydata = args_["custom-config"].as<std::string>().c_str();
            } else {
                ROS_WARN("Custom board enabled but 'custom-config' string is empty.");
            }
        }

        params.parameters = parameter_string.c_str();
        params.protocol = "camera.gmsl";

        ROS_INFO("Creating sensor with parameters: %s", params.parameters);
        CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
        ROS_INFO("Sensor created. Starting sensor...");
        CHECK_DW_ERROR(dwSensor_start(camera_));
        ROS_INFO("Sensor started. Waiting for first frame...");

        // Wait for camera to be ready (Simplified init check)
        dwCameraFrameHandle_t frame = DW_NULL_HANDLE;
        dwStatus status = DW_NOT_AVAILABLE; // Start with a non-success status
        int retries = 10;
        while ((status == DW_NOT_READY || status == DW_TIME_OUT || status == DW_NOT_AVAILABLE) && retries-- > 0) {
            ros::Duration(0.2).sleep();
            status = dwSensorCamera_readFrame(&frame, 0, 200000, camera_);
            if (status == DW_SUCCESS && frame != DW_NULL_HANDLE) {
                 ROS_INFO("First frame read successfully during init.");
                // Check format of the first frame (Best effort)
                 dwImageNvMedia* nvmedia_ptr = nullptr;
                 dwStatus imgStatus = dwSensorCamera_getImageNvMedia(&nvmedia_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame);
                 if(imgStatus == DW_SUCCESS && nvmedia_ptr && nvmedia_ptr->img) {
                     dwImageHandle_t tmp_h = DW_NULL_HANDLE;
                     if(dwImage_createAndBindNvMedia(&tmp_h, nvmedia_ptr->img) == DW_SUCCESS){
                         dwImageProperties props;
                         if(dwImage_getProperties(&props, tmp_h) == DW_SUCCESS) {
                             ROS_INFO("Init Frame Properties: Format=%d (%s), W=%u, H=%u",
                                      props.format, dwImageFormatToStr(props.format), props.width, props.height);
                         }
                         dwImage_destroy(&tmp_h);
                     }
                 } else {
                     ROS_WARN("Could not get NvMedia Image details from first frame in init (Status: %s)", dwGetStatusName(imgStatus));
                 }
                 CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
                 break; // Exit loop on success
            } else {
                 ROS_WARN("Camera init read status: %s (Retries left: %d)", dwGetStatusName(status), retries);
                 if (frame) { // If handle was acquired but status is error, return it
                     dwSensorCamera_returnFrame(&frame);
                     frame = DW_NULL_HANDLE;
                 }
            }
        }
        if (status != DW_SUCCESS) {
             ROS_ERROR("Final camera status after init retries: %s", dwGetStatusName(status));
             if (camera_) { dwSAL_releaseSensor(&camera_); camera_ = DW_NULL_HANDLE; }
             throw std::runtime_error("Camera did not start correctly.");
        }

        CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
        ROS_INFO("Successfully initialized camera. Native resolution reported: %dx%d@%f FPS\n",
                 camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);

        // Initialize the *final* RGBA image handle with TARGET resolution
        dwImageProperties rgba_resized_prop{};
        rgba_resized_prop.height = TARGET_HEIGHT;
        rgba_resized_prop.width = TARGET_WIDTH;
        rgba_resized_prop.type = DW_IMAGE_NVMEDIA;
        rgba_resized_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8; // Target format is RGBA
        CHECK_DW_ERROR(dwImage_create(&frame_rgba_resized_, rgba_resized_prop, sdk_));
        ROS_INFO("Initialized NvMedia RGBA image handle for target %dx%d.\n", TARGET_WIDTH, TARGET_HEIGHT);
    }

    ~CameraGMSL() {
         ROS_INFO("Destructor called.");
         if (camera_) {
            dwSensor_stop(camera_); // Ignore stop errors?
            dwSAL_releaseSensor(&camera_);
            camera_ = DW_NULL_HANDLE;
         }
         if (frame_rgba_resized_) {
            dwImage_destroy(&frame_rgba_resized_);
            frame_rgba_resized_ = DW_NULL_HANDLE;
         }
         if (sal_) { dwSAL_release(&sal_); sal_ = DW_NULL_HANDLE; }
         if (sdk_) { dwRelease(&sdk_); sdk_ = DW_NULL_HANDLE; }
         ROS_INFO("Resources released.");
    }

    void publish()
    {
        ROS_INFO("Starting GPU-accelerated publishing (YUV input -> RGBA resize) with target %dx%d", TARGET_WIDTH, TARGET_HEIGHT);

        try
        {
            ros::Rate loop_rate(15);
            int count = 0;
            bool first_conversion_attempt = true;

            while (ros::ok())
            {
                dwTime_t timeout = 132000;
                dwCameraFrameHandle_t frame_handle = DW_NULL_HANDLE;
                dwImageHandle_t frame_yuv_wrapper = DW_NULL_HANDLE; // Wrapper for the camera's NvMedia YUV
                dwImageNvMedia* nvmedia_yuv_ptr = nullptr;
                dwImageNvMedia* nvmedia_rgba_resized_ptr = nullptr;

                sensor_msgs::ImagePtr ros_img_ptr = boost::make_shared<sensor_msgs::Image>();
                std_msgs::Header header;
                header.seq = count;
                header.stamp = ros::Time::now();
                header.frame_id = "camera_frame";

                // 1. Read frame (expecting YUV)
                dwStatus read_status = dwSensorCamera_readFrame(&frame_handle, 0, timeout, camera_);
                if (read_status != DW_SUCCESS) {
                    ROS_WARN_THROTTLE(1.0, "Failed or timeout reading frame %d: %s", count, dwGetStatusName(read_status));
                    if(frame_handle) { dwSensorCamera_returnFrame(&frame_handle); }
                    continue;
                }
                if (!frame_handle) { ROS_ERROR("[%d] Read success but null handle!", count); continue;}

                // 2. Get NvMedia handle for the native YUV image
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));
                if (!nvmedia_yuv_ptr || !nvmedia_yuv_ptr->img) {
                     ROS_ERROR("[%d] getImageNvMedia (YUV) success but null ptr/img!", count);
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));
                     continue;
                 }

                // 3. Create temporary DW handle bound to the YUV NvMedia image
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv_wrapper, nvmedia_yuv_ptr->img));

                // ***** DEBUG: Log YUV properties BEFORE conversion attempt *****
                if (first_conversion_attempt) { // Only log for the first frame or on error retry
                    dwImageProperties yuv_props;
                    dwStatus prop_status = dwImage_getProperties(&yuv_props, frame_yuv_wrapper);
                    if (prop_status == DW_SUCCESS) {
                        ROS_INFO("Attempting conversion FROM: Format=%d (%s), W=%u, H=%u, Type=%d",
                                 yuv_props.format, dwImageFormatToStr(yuv_props.format),
                                 yuv_props.width, yuv_props.height, yuv_props.type);
                         ROS_INFO("Attempting conversion TO: Format=%d (%s), W=%u, H=%u, Type=%d",
                                 DW_IMAGE_FORMAT_RGBA_UINT8, dwImageFormatToStr(DW_IMAGE_FORMAT_RGBA_UINT8),
                                 TARGET_WIDTH, TARGET_HEIGHT, DW_IMAGE_NVMEDIA); // Target properties
                    } else {
                        ROS_ERROR("Failed to get properties of source YUV wrapper: %s", dwGetStatusName(prop_status));
                    }
                }
                // ***** END DEBUG *****

                // 4. ***** Perform GPU-accelerated Conversion AND Resizing *****
                dwStatus convertStatus = dwImage_copyConvert(frame_rgba_resized_, frame_yuv_wrapper, sdk_);
                if (convertStatus != DW_SUCCESS) {
                    ROS_ERROR("[%d] dwImage_copyConvert FAILED: %s", count, dwGetStatusName(convertStatus));
                    // Log again if it fails after the first time
                    if (!first_conversion_attempt) {
                         dwImageProperties yuv_props;
                         if (dwImage_getProperties(&yuv_props, frame_yuv_wrapper) == DW_SUCCESS) {
                            ROS_ERROR("Failed conversion FROM: Format=%d (%s), W=%u, H=%u",
                                      yuv_props.format, dwImageFormatToStr(yuv_props.format), yuv_props.width, yuv_props.height);
                         }
                    }
                    first_conversion_attempt = true; // Reset flag to log details on next attempt if it continues
                    // Cleanup and continue to next frame
                    CHECK_DW_ERROR(dwImage_destroy(&frame_yuv_wrapper));
                    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));
                    continue; // Skip rest of the loop for this frame
                }
                // If conversion succeeds, reset the flag
                first_conversion_attempt = false;


                // 5. Get NvMedia handle for the final resized RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgba_resized_ptr, frame_rgba_resized_));
                if (!nvmedia_rgba_resized_ptr || !nvmedia_rgba_resized_ptr->img) {
                     ROS_ERROR("[%d] getNvMedia (RGBA resized) success but null ptr/img!", count);
                     CHECK_DW_ERROR(dwImage_destroy(&frame_yuv_wrapper));
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));
                     continue;
                }


                // 6. Access NvMedia RGBA data
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgba_resized_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // 7. Populate ROS message
                    ros_img_ptr->header = header;
                    ros_img_ptr->height = TARGET_HEIGHT;
                    ros_img_ptr->width = TARGET_WIDTH;
                    ros_img_ptr->encoding = sensor_msgs::image_encodings::RGBA8;
                    ros_img_ptr->is_bigendian = false;
                    ros_img_ptr->step = static_cast<uint32_t>(TARGET_WIDTH * 4);
                    size_t img_size = static_cast<size_t>(ros_img_ptr->step) * TARGET_HEIGHT;
                    ros_img_ptr->data.resize(img_size);

                    // 8. Copy data GPU -> CPU
                    size_t expected_pitch = static_cast<size_t>(TARGET_WIDTH) * 4;
                    if (surfaceMap.surface[0].mapping == nullptr) {
                        ROS_ERROR("[%d] NvMediaImageLock ok but mapping is NULL!", count);
                    } else if (surfaceMap.surface[0].pitch == expected_pitch) {
                        memcpy(ros_img_ptr->data.data(), surfaceMap.surface[0].mapping, img_size);
                    } else {
                        ROS_WARN_ONCE("NvMedia surface pitch (%zu) != expected (%zu). Copying row by row.",
                                      (size_t)surfaceMap.surface[0].pitch, expected_pitch);
                        uint8_t* ros_data_ptr = ros_img_ptr->data.data();
                        uint8_t* nv_data_ptr = static_cast<uint8_t*>(surfaceMap.surface[0].mapping);
                        for (uint32_t row = 0; row < TARGET_HEIGHT; ++row) {
                            memcpy(ros_data_ptr + row * expected_pitch,
                                   nv_data_ptr + row * surfaceMap.surface[0].pitch,
                                   expected_pitch);
                        }
                    }

                    // 9. Publish (only if data seemed valid)
                    if(surfaceMap.surface[0].mapping != nullptr) {
                       gmsl_pub_img_.publish(ros_img_ptr);
                    }

                    // 10. Unlock
                    NvMediaImageUnlock(nvmedia_rgba_resized_ptr->img);
                } else {
                     ROS_WARN("Failed to lock NvMedia image for reading frame %d.", count);
                }

                // 11. Cleanup
                if (frame_yuv_wrapper) { CHECK_DW_ERROR(dwImage_destroy(&frame_yuv_wrapper)); }
                if (frame_handle) { CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle)); }

                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            }
        }
        catch (const std::runtime_error &e) { ROS_ERROR("Runtime error: %s", e.what()); }
        catch (const std::exception &e) { ROS_ERROR("Standard exception: %s", e.what()); }
        catch (...) { ROS_ERROR("Unknown exception caught."); }
        ROS_INFO("Publish loop finished.");
    }
};

// main function remains the same as the previous C++11 compatible version
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_gmsl_gpu_convert_resize"); // Node name reflecting strategy

    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"), "Camera GMSL type")
        ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [a|c|e|g]")
        ("tegra-slave", po::value<std::string>()-> default_value("0"), "Optional: Tegra B slave mode [0|1]")
        ("custom-board", po::value<std::string>()-> default_value("0"), "Use custom board config [0|1]")
        ("custom-config", po::value<std::string>()-> default_value(""), "Path or string for custom board config");

    po::variables_map args;
    try {
        po::store(po::parse_command_line(argc, const_cast<const char**>(argv), desc), args);
        if (args.count("help")) { std::cout << desc << std::endl; return 0; }
        po::notify(args);
    } catch (const po::error &ex) {
        ROS_ERROR("Error parsing options: %s", ex.what()); std::cerr << desc << std::endl; return 1;
    } catch (const std::exception &ex) {
        ROS_ERROR("Error initializing options: %s", ex.what()); return 1;
    }

    std::unique_ptr<CameraGMSL> cam_ptr;
    try {
        ROS_INFO("Creating CameraGMSL object (YUV input, GPU convert+resize)...");
        cam_ptr.reset(new CameraGMSL(args)); // C++11 style unique_ptr
        ROS_INFO("CameraGMSL object created. Starting publishing...");
        cam_ptr->publish();
    } catch (const std::runtime_error &e) {
        ROS_FATAL("Initialization or runtime error: %s", e.what()); return 1;
    } catch (const std::exception& e) {
        ROS_FATAL("Unexpected standard error: %s", e.what()); return 1;
    } catch (...) {
        ROS_FATAL("Unknown error during setup or run."); return 1;
    }

    ROS_INFO("Camera GMSL node shutting down normally.");
    return 0;
}
