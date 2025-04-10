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
    
    // Half resolution dimensions
    uint32_t half_width_;
    uint32_t half_height_;

    // Buffer for the half-resolution image
    unsigned char* half_res_buffer_ = nullptr;
    size_t half_res_buffer_size_ = 0;

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

            parameter_string += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

            if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                // it's a custom board, use the board specific extra configurations
                parameter_string += ",custom-board=1";

                // pass an extra set of parameter in custom-config
                params.auxiliarydata = args_["custom-board"].as<std::string>().c_str();
            }

            params.parameters = parameter_string.c_str();
            params.protocol = "camera.gmsl";

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
            ROS_INFO("Successfully initialized camera with resolution of %dx%d at framerate of %f FPS\n",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);
                
            // Calculate half resolution dimensions
            half_width_ = camera_properties_.resolution.x / 2;
            half_height_ = camera_properties_.resolution.y / 2;
        }

        //ROS initialization
        {
            ros::VP_string ros_str;
            ros::init(ros_str, "camera_gmsl");
            ros::NodeHandle n;
            gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);

            ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
            ROS_INFO("Successfully initialized ros\n");
        }

        //Nvmedia initialization
        {
            dwImageProperties rgb_img_prop{};
            rgb_img_prop.height = camera_properties_.resolution.y;
            rgb_img_prop.width = camera_properties_.resolution.x;
            rgb_img_prop.type = DW_IMAGE_NVMEDIA;
            rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            CHECK_DW_ERROR(dwImage_create(&frame_rgb_, rgb_img_prop, sdk_));
            
            // Allocate buffer for half-resolution image (RGBA format = 4 bytes per pixel)
            half_res_buffer_size_ = half_width_ * half_height_ * 4;
            half_res_buffer_ = new unsigned char[half_res_buffer_size_];
            
            ROS_INFO("Successfully initialized nvmedia: Original resolution: %dx%d, Half resolution: %dx%d\n", 
                     camera_properties_.resolution.x, camera_properties_.resolution.y,
                     half_width_, half_height_);
        }
    }

    ~CameraGMSL()
    {
        ROS_INFO("Destructor!!!");
        if (camera_) {
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
        }

        //destroy created image
        dwImage_destroy(&frame_rgb_);
        
        // Free half resolution buffer
        if(half_res_buffer_) {
            delete[] half_res_buffer_;
            half_res_buffer_ = nullptr;
        }

        dwSAL_release(&sal_);
        dwRelease(&sdk_);
        dwLogger_release();
    }
    
    // Helper function to downsample an image by averaging 2x2 blocks
    void downsampleRGBA(const unsigned char* src, unsigned char* dst, int srcWidth, int srcHeight)
    {
        int dstWidth = srcWidth / 2;
        int dstHeight = srcHeight / 2;
        
        for (int y = 0; y < dstHeight; y++) {
            for (int x = 0; x < dstWidth; x++) {
                int srcX = x * 2;
                int srcY = y * 2;
                
                // Average 2x2 block of pixels for each channel (R,G,B,A)
                for (int c = 0; c < 4; c++) {
                    int srcPos1 = (srcY * srcWidth + srcX) * 4 + c;
                    int srcPos2 = (srcY * srcWidth + srcX + 1) * 4 + c;
                    int srcPos3 = ((srcY + 1) * srcWidth + srcX) * 4 + c;
                    int srcPos4 = ((srcY + 1) * srcWidth + srcX + 1) * 4 + c;
                    
                    int sum = src[srcPos1] + src[srcPos2] + src[srcPos3] + src[srcPos4];
                    dst[(y * dstWidth + x) * 4 + c] = (unsigned char)(sum / 4);
                }
            }
        }
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
                
                // read from camera will update the low level buffers frame of the camera
                // those frames are images with NATIVE properties that depend on the type and sensor properties set at creation
                CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, camera_sibling_id, timeout, camera_));

                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
                
                // Get the original resolution image data
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgb_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    unsigned char* buffer = (unsigned char*)surfaceMap.surface[0].mapping;
                    
                    // Downsample the image (CPU implementation since GPU version isn't available)
                    downsampleRGBA(buffer, half_res_buffer_, 
                                  nvmedia_rgb_img_ptr->prop.width, 
                                  nvmedia_rgb_img_ptr->prop.height);
                    
                    NvMediaImageUnlock(nvmedia_rgb_img_ptr->img);
                    
                    // Setup ROS message
                    header.seq = count; // user defined counter
                    header.stamp = ros::Time::now(); 
                        
                    img_msg.header = header;
                    img_msg.height = half_height_;
                    img_msg.width = half_width_;
                    img_msg.encoding = sensor_msgs::image_encodings::RGBA8;
                    
                    img_msg.step = img_msg.width * 4; // 1 Byte per 4 Channels of the RGBA format

                    img_size = img_msg.step * img_msg.height;
                    img_msg.data.resize(img_size);
                    
                    // Copy the downsampled image to the ROS message
                    memcpy((char *)(&img_msg.data[0]), half_res_buffer_, img_size);
                    gmsl_pub_img_.publish(ros_img_ptr_);
                }
                
                // Cleanup
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv));       
                // return frame
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
        ("custom-config", po::value<std::string>()-> default_value(""), "Set of custom board extra configuration\n");

    po::variables_map args;
    po::store(parse_command_line(argc, argv, desc), args);
    po::notify(args);

    CameraGMSL cam(args);
    cam.publish();    

    return 0;
}
