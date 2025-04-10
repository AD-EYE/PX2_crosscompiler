// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ros/console.h"

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/Status.h>
#include <dw/sensors/Sensors.h>

// HAL
#include <dw/sensors/camera/Camera.h>
#include <dw/sensors/camera/CameraDevice.h>
#include <dw/sensors/plugins/camera/Camera.h>

// Image
#include <dw/image/Image.h>
#include <dw/image/ImageStreamer.h>

// nvmedia for surface map
#include <cuda_runtime.h>
#include "nvmedia_image.h"
#include "nvmedia_ijpe.h"
#include "nvmedia_surface.h"

#include <memory>
#include <string>
#include <stdexcept>
#include <iostream>
#include <boost/program_options.hpp>

#define CHECK_DW_ERROR(x) { \
    dwStatus result = x; \
    if(result!=DW_SUCCESS) { \
        throw std::runtime_error(std::string("DW Error ") \
        + dwGetStatusName(result) \
        + std::string(" executing DW function:\n " #x) \
        + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
    }};

#define CHECK_CUDA_ERROR(x) { \
    cudaError_t error = x; \
    if(error!=cudaSuccess) { \
        throw std::runtime_error(std::string("CUDA Error: ") \
        + cudaGetErrorString(error) \
        + std::string(" executing CUDA function:\n " #x) \
        + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
    }};

namespace po = boost::program_options;

class CameraGMSL
{
private:
    // Driveworks Context and SAL
    dwContextHandle_t sdk_ = DW_NULL_HANDLE;
    dwSALHandle_t sal_ = DW_NULL_HANDLE;
    
    // ROS variables
    ros::Publisher gmsl_pub_img_;
    sensor_msgs::ImagePtr ros_img_ptr_;
    
    // Image handles and properties
    dwImageHandle_t frame_rgb_ = DW_NULL_HANDLE;
    dwImageStreamerHandle_t cuda2rgba_ = DW_NULL_HANDLE;
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwImageProperties camera_image_properties_;
    dwCameraProperties camera_properties_;
    
    // CUDA memory management
    cudaStream_t cudaStream_;
    void* devPtr_ = nullptr;
    void* hostPtr_ = nullptr;
    
    po::variables_map args_;
    
public:
    CameraGMSL(const po::variables_map args) : args_(args)
    {
        // Initialize DriveWorks context and SAL
        dwContextParameters sdk_params = {};
        CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));
        
        // GPU device seçimi: Performans için dGPU (0) kullanılıyor
        CHECK_DW_ERROR(dwContext_selectGPUDevice(sdk_, 0));
        
        // CUDA stream oluştur
        CHECK_CUDA_ERROR(cudaStreamCreate(&cudaStream_));
        
        // HAL modülünü başlat
        CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
        
        // GMSL kamera ayarları
        initializeCamera();
        
        // ROS ayarları
        initializeROS();
        
        // GPU bazlı görüntü işleme için streamer
        initializeImageStreamer();
    }
    
    ~CameraGMSL()
    {
        ROS_INFO("Cleaning up resources...");
        
        if (camera_) {
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
        }
        
        // GPU belleği temizle
        if (devPtr_) {
            CHECK_CUDA_ERROR(cudaFree(devPtr_));
        }
        
        // Pinned host belleği temizle
        if (hostPtr_) {
            CHECK_CUDA_ERROR(cudaFreeHost(hostPtr_));
        }
        
        // CUDA stream yok et
        CHECK_CUDA_ERROR(cudaStreamDestroy(cudaStream_));
        
        // Streamer'ı yok et
        if (cuda2rgba_ != DW_NULL_HANDLE) {
            dwImageStreamer_release(&cuda2rgba_);
        }
        
        // Image yok et
        if (frame_rgb_ != DW_NULL_HANDLE) {
            dwImage_destroy(&frame_rgb_);
        }
        
        // DriveWorks bağlamını temizle
        dwSAL_release(&sal_);
        dwRelease(&sdk_);
        dwLogger_release();
    }
    
    void initializeCamera()
    {
        // Kamera parametreleri
        dwSensorParams params;
        std::string parameter_string = std::string("output-format=yuv,fifo-size=3");
        parameter_string += std::string(",camera-type=") + args_["camera-type"].as<std::string>();
        parameter_string += std::string(",csi-port=") + args_["camera-port"].as<std::string>();
        parameter_string += std::string(",slave=") + args_["tegra-slave"].as<std::string>();
        
        if (args_["custom-board"].as<std::string>().compare("1") == 0) {
            // Özel board konfigürasyonu
            parameter_string += ",custom-board=1";
            params.auxiliarydata = args_["custom-board"].as<std::string>().c_str();
        }
        
        params.parameters = parameter_string.c_str();
        params.protocol = "camera.gmsl";
        
        // Kamera sensörünü oluştur
        CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
        CHECK_DW_ERROR(dwSensor_start(camera_));
        
        // Kameranın başlatılmasını bekle
        dwCameraFrameHandle_t frame;
        dwStatus status = DW_NOT_READY;
        do {
            status = dwSensorCamera_readFrame(&frame, 0, 66000, camera_);
        } while (status == DW_NOT_READY);
        
        if (status != DW_SUCCESS) {
            throw std::runtime_error("Cameras did not start correctly");
        }
        
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
        CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
        
        ROS_INFO("Successfully initialized camera with resolution of %dx%d at framerate of %f FPS",
                 camera_properties_.resolution.x, camera_properties_.resolution.y, 
                 camera_properties_.framerate);
    }
    
    void initializeROS()
    {
        ros::VP_string ros_str;
        ros::init(ros_str, "camera_gmsl");
        ros::NodeHandle n;
        gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);
        ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
        ROS_INFO("Successfully initialized ROS");
    }
    
    void initializeImageStreamer()
    {
        // GPU üzerinde RGBA görüntü için bellek ayırma
        dwImageProperties rgb_img_prop{};
        rgb_img_prop.height = camera_properties_.resolution.y;
        rgb_img_prop.width = camera_properties_.resolution.x;
        rgb_img_prop.type = DW_IMAGE_NVMEDIA;
        rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
        
        CHECK_DW_ERROR(dwImage_create(&frame_rgb_, rgb_img_prop, sdk_));
        
        // CUDA to RGBA streamer oluştur (GPU içi transfer için)
        dwImageStreamerParameters streamer_params{};
        streamer_params.outputType = DW_IMAGE_CPU;
        streamer_params.maxOutputWidth = camera_properties_.resolution.x;
        streamer_params.maxOutputHeight = camera_properties_.resolution.y;
        streamer_params.cudaStream = cudaStream_;
        
        CHECK_DW_ERROR(dwImageStreamer_initialize(&cuda2rgba_, &streamer_params, sdk_));
        
        // Pinned bellek ayırma (hızlı CPU-GPU transfer için)
        size_t img_size = camera_properties_.resolution.x * 
                           camera_properties_.resolution.y * 4; // RGBA için 4 byte
        
        CHECK_CUDA_ERROR(cudaMallocHost(&hostPtr_, img_size));
        CHECK_CUDA_ERROR(cudaMalloc(&devPtr_, img_size));
        
        ROS_INFO("Successfully initialized GPU image processing resources");
    }
    
    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type - %s", cam_type.c_str());
        ROS_INFO("Starting to publish images (GPU optimized)");
        
        try {
            ros::Rate loop_rate(15);
            int count = 0;
            
            while (ros::ok()) {
                dwTime_t timeout = 132000;
                dwCameraFrameHandle_t frame;
                uint32_t camera_sibling_id = 0;
                
                // GPU üzerinde görüntü işlemek için gerekli değişkenler
                dwImageHandle_t frame_yuv;
                dwImageHandle_t cuda_frame;
                dwImageCUDA* cuda_image_ptr;
                dwImageNvMedia* nvmedia_yuv_img_ptr;
                dwImageNvMedia* nvmedia_rgb_img_ptr;
                
                // İleti başlığı
                std_msgs::Header header;
                header.seq = count++;
                header.stamp = ros::Time::now();
                
                // Kameradan frame okuma (native)
                CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, camera_sibling_id, timeout, camera_));
                
                // YUV NvMedia görüntüsünü al
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, 
                                                    DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));
                
                // CUDA formatına doğrudan erişim (GPU üzerinde kalır)
                CHECK_DW_ERROR(dwSensorCamera_getImage(&cuda_frame, 
                                                    DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, frame));
                CHECK_DW_ERROR(dwImage_getCUDA(&cuda_image_ptr, cuda_frame));
                
                // RGBA görüntüye NVMEDIA erişimi (GPU'da)
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
                
                // YUV'dan RGBA'ya dönüşüm (tamamen GPU üzerinde)
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));
                
                // Streamer ile CUDA görüntüyü CPU'ya transfer et (asenkron)
                dwImageHandle_t streamed_rgba = DW_NULL_HANDLE;
                CHECK_DW_ERROR(dwImageStreamer_producerSend(frame_rgb_, cuda2rgba_));
                CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&streamed_rgba, 33000, cuda2rgba_));
                
                // Görüntü verilerini ROS iletisine kopyala
                dwImageCPU* cpu_image;
                CHECK_DW_ERROR(dwImage_getCPU(&cpu_image, streamed_rgba));
                
                // ROS ileti formatını doldur
                sensor_msgs::Image &img_msg = *ros_img_ptr_;
                img_msg.header = header;
                img_msg.height = cpu_image->prop.height;
                img_msg.width = cpu_image->prop.width;
                img_msg.encoding = sensor_msgs::image_encodings::RGBA8;
                img_msg.step = img_msg.width * 4;
                
                size_t img_size = img_msg.step * img_msg.height;
                img_msg.data.resize(img_size);
                
                // zero-copy için pinned bellek kullanımı
                memcpy(&img_msg.data[0], cpu_image->data[0], img_size);
                
                // ROS üzerinden yayınla
                gmsl_pub_img_.publish(ros_img_ptr_);
                
                // Kaynakları serbest bırak
                CHECK_DW_ERROR(dwImageStreamer_consumerReturn(streamed_rgba, cuda2rgba_));
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv));
                CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
                
                ros::spinOnce();
                loop_rate.sleep();
            }
        } catch (std::runtime_error &e) {
            std::cerr << e.what() << std::endl;
        }
    }
};

int main(int argc, const char *argv[])
{
    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()->default_value("ar0231-rccb-bae-sf3324"),
           "camera gmsl type (see sample_sensors_info for all available camera types on this platform)")
        ("camera-port", po::value<std::string>()->default_value("a"), "Camera CSI port [default a]\n"
           "a - port AB on px2, A on ddpx\n"
           "c - port CD on px2, C on ddpx\n"
           "e - port EF on px2, E on ddpx\n"
           "g - G on ddpx only")
        ("tegra-slave", po::value<std::string>()->default_value("0"),
           "Optional parameter used only for Tegra B, enables slave mode.")
        ("custom-board", po::value<std::string>()->default_value("0"), "If true, then the configuration for board and camera "
           "will be input from the config-file")
        ("custom-config", po::value<std::string>()->default_value(""), "Set of custom board extra configuration");
    
    po::variables_map args;
    po::store(parse_command_line(argc, argv, desc), args);
    po::notify(args);
    
    if (args.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    
    try {
        CameraGMSL cam(args);
        cam.publish();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

