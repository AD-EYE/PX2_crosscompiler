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
    ros::Publisher gmsl_pub_, gmsl_pub_img_;
    sensor_msgs::ImagePtr ros_img_ptr_; 

    // Image handles and properties
    dwImageHandle_t frame_rgb_ = DW_NULL_HANDLE; 
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwImageProperties camera_image_properties_;
    dwCameraProperties camera_properties_;

    // Resolution properties
    uint32_t output_width_;
    uint32_t output_height_;
    float resolution_ratio_;

    po::variables_map args_;
     
public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        // Get resolution ratio parameter (between 0.0 and 1.0)
        resolution_ratio_ = args_["resolution-ratio"].as<float>();
        
        // Validate the ratio is between 0.0 and 1.0
        if (resolution_ratio_ <= 0.0f || resolution_ratio_ > 1.0f) {
            ROS_WARN("Invalid resolution ratio %.2f. Must be between 0.0 and 1.0. Using 1.0 (full resolution).", resolution_ratio_);
            resolution_ratio_ = 1.0f;
        }
        
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
            
            // Calculate output resolution based on ratio while maintaining aspect ratio
            output_width_ = static_cast<uint32_t>(camera_properties_.resolution.x * resolution_ratio_);
            output_height_ = static_cast<uint32_t>(camera_properties_.resolution.y * resolution_ratio_);
            
            // Ensure minimum size of 1x1
            output_width_ = std::max(output_width_, 1u);
            output_height_ = std::max(output_height_, 1u);
            
            ROS_INFO("Camera native resolution: %dx%d at framerate of %f FPS",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);
            ROS_INFO("Using resolution ratio: %.2f", resolution_ratio_);
            ROS_INFO("Output resolution: %dx%d", output_width_, output_height_);
        }

        // ROS initialization
        {
            ros::VP_string ros_str;
            ros::init(ros_str, "camera_gmsl");
            ros::NodeHandle n;
            gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw_cl1rev4", 1);

            ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
            ROS_INFO("Successfully initialized ros\n");
        }

        // Create RGBA image with the desired output resolution
        {
            dwImageProperties rgb_img_prop{};
            rgb_img_prop.height = output_height_;
            rgb_img_prop.width = output_width_;
            rgb_img_prop.type = DW_IMAGE_NVMEDIA;
            rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            CHECK_DW_ERROR(dwImage_create(&frame_rgb_, rgb_img_prop, sdk_));
            ROS_INFO("Successfully initialized nvmedia img with resolution %dx%d\n", 
                    output_width_, output_height_);
        }
    }

    ~CameraGMSL()
    {
        ROS_INFO("Destructor!!!");
        if (camera_) {
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
        }

        // Destroy created image
        dwImage_destroy(&frame_rgb_);

        dwSAL_release(&sal_);
        dwRelease(&sdk_);
        dwLogger_release();
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type - %s \n", cam_type.c_str());
        ROS_INFO("Starting to publish images");

        try
        {
            ros::Rate loop_rate(15);
            int count = 0;
            while (ros::ok())
            {
                dwTime_t timeout = 132000; 
                dwCameraFrameHandle_t frame;
                uint32_t camera_sibling_id = 0;
                dwImageHandle_t frame_yuv;
                dwImageNvMedia* nvmedia_yuv_img_ptr;
                dwImageNvMedia* nvmedia_rgb_img_ptr;

                sensor_msgs::Image &img_msg = *ros_img_ptr_; // >> message to be sent
                std_msgs::Header header; // empty header
                size_t img_size;
                
                // Read from camera
                CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, camera_sibling_id, timeout, camera_));

                // Get the YUV image from camera
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));
                
                // Bind YUV image to a frame handle
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));
                
                // Get the RGBA image handle
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
                
                // Try to use copyConvert and see if it handles the different dimensions
                // In DriveWorks 1.2 this might do the scaling automatically
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));
                
                // Prepare ROS message
                header.seq = count; // user defined counter
                header.stamp = ros::Time::now(); 
                    
                img_msg.header = header;
                img_msg.height = nvmedia_rgb_img_ptr->prop.height;
                img_msg.width = nvmedia_rgb_img_ptr->prop.width;
                img_msg.encoding = sensor_msgs::image_encodings::RGBA8;
                
                img_msg.step = img_msg.width * 4; // 1 Byte per 4 Channels of the RGBA format

                img_size = img_msg.step * img_msg.height;
                img_msg.data.resize(img_size);
                
                // Copy data from GPU to CPU
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgb_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    unsigned char* buffer = (unsigned char*)surfaceMap.surface[0].mapping;
                    memcpy((char *)(&img_msg.data[0]), buffer, img_size);
                    gmsl_pub_img_.publish(ros_img_ptr_);
                    NvMediaImageUnlock(nvmedia_rgb_img_ptr->img);
                }
                
                // Cleanup
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv));
                
                // Return frame to camera
                CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
                
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            }
        }
        catch (std::runtime_error &e)
        {
            std::cerr << e.what() << "\n";
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char *argv[])
{
    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"), 
            "camera gmsl type (see sample_sensors_info for all available camera types on this platform)\n")
        ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [default a]\n"
                              "a - port AB on px2, A on ddpx\n"
                              "c - port CD on px2, C on ddpx\n"
                              "e - port EF on px2, E on ddpx\n"
                              "g - G on ddpx only\n")
        ("tegra-slave", po::value<std::string>()-> default_value("0"),
            "Optional parameter used only for Tegra B, enables slave mode.\n")
        ("custom-board", po::value<std::string>()-> default_value("0"), "If true, then the configuration for board and camera "
                              "will be input from the config-file\n")
        ("custom-config", po::value<std::string>()-> default_value(""), "Set of custom board extra configuration\n")
        ("resolution-ratio", po::value<float>()-> default_value(1.0f), 
            "Resolution scaling ratio (0.0 to 1.0). For example, 0.5 means half the native resolution.\n");

    po::variables_map args;
    po::store(parse_command_line(argc, argv, desc), args);
    po::notify(args);

    CameraGMSL cam(args);
    cam.publish();    

    return 0;
}
