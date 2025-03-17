
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
#include <cv_bridge/cv_bridge.h>


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
            ROS_INFO("Successfully initialized camera with resolution of %dx%d at framerate of %f FPS\n",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);

        }

         //ROS initialization
        {
            ros::VP_string ros_str;
            ros::init(ros_str, "camera_gmsl");
            ros::NodeHandle n;
            gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);

            ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
            ROS_INFO("Successfully initialized ros\n" );
        }

        //Nvmedia initialization
        {
            dwImageProperties rgb_img_prop{};
            rgb_img_prop.height = camera_properties_.resolution.y;
            rgb_img_prop.width = camera_properties_.resolution.x;
            rgb_img_prop.type = DW_IMAGE_NVMEDIA;
            rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            CHECK_DW_ERROR(dwImage_create(&frame_rgb_,  rgb_img_prop,sdk_	));
            ROS_INFO("Successfully initialized nvmedia img.\n" );
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

        dwSAL_release(&sal_);
        dwRelease(&sdk_);
        dwLogger_release();

    }

void publish()
{
    std::string cam_type = args_["camera-type"].as<std::string>();
    ROS_INFO("Camera type - %s \n", cam_type.c_str());
    ROS_INFO("Starting to publish images");

    // Define your target resolution here
    const int TARGET_WIDTH = 640;  // Change to your desired width
    const int TARGET_HEIGHT = 480; // Change to your desired height

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

            // read from camera
            CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, camera_sibling_id, timeout, camera_));

            // Get YUV frame and convert to RGB as before
            CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));
            CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
            CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));
            CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));
            CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));

            // Create header for ROS message
            std_msgs::Header header;
            header.seq = count;
            header.stamp = ros::Time::now(); 
            
            // Lock NvMedia image to access its data
            NvMediaImageSurfaceMap surfaceMap;
            if (NvMediaImageLock(nvmedia_rgb_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
            {
                // Get image dimensions
                int original_height = nvmedia_rgb_img_ptr->prop.height;
                int original_width = nvmedia_rgb_img_ptr->prop.width;
                
                // Create OpenCV Mat from NvMedia buffer (RGBA format)
                cv::Mat original_image(original_height, original_width, CV_8UC4, surfaceMap.surface[0].mapping);
                
                // Create a destination Mat with the target resolution
                cv::Mat resized_image;
                cv::resize(original_image, resized_image, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_LINEAR);
                
                // Convert OpenCV Mat to ROS Image message
                sensor_msgs::ImagePtr resized_msg = cv_bridge::CvImage(header, 
                                                                     sensor_msgs::image_encodings::RGBA8, 
                                                                     resized_image).toImageMsg();
                
                // Publish the resized image
                gmsl_pub_img_.publish(resized_msg);
                
                // Unlock NvMedia image
                NvMediaImageUnlock(nvmedia_rgb_img_ptr->img);
            }
            
            // Cleanup
            CHECK_DW_ERROR(dwImage_destroy(&frame_yuv));       
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
