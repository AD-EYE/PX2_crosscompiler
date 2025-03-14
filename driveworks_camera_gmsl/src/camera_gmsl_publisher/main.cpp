#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>   // DW_VERSION tanımı için gerekli
#include <dw/core/NvMedia.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/ImageStreamer.h>
#include <dw/image/FormatConverter_vibrante.h>

#include <nvmedia_2d.h>
#include "nvmedia_image.h"
#include "nvmedia_ijpe.h"
#include "nvmedia_surface.h"

#include <dw/renderer/Renderer.h>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>

#define CHECK_DW_ERROR(x) { \
    dwStatus result = x; \
    if(result != DW_SUCCESS) { \
        throw std::runtime_error(std::string("DW Error ") + dwGetStatusName(result) + " executing DW function: " #x); \
    } \
}

namespace po = boost::program_options;

class CameraGMSL {
private:
    dwContextHandle_t sdk_ = DW_NULL_HANDLE;
    dwSALHandle_t sal_ = DW_NULL_HANDLE;
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwCameraProperties camera_properties_;

    ros::Publisher gmsl_pub_img_;
    sensor_msgs::ImagePtr ros_img_ptr_;

    // Native görüntü, sensörden alınan orijinal çözünürlükte (örneğin 1920x1208) oluşturuluyor
    dwImageHandle_t frame_rgb_native_ = DW_NULL_HANDLE;

    po::variables_map args_;

public:
    CameraGMSL(const po::variables_map &args) : args_(args) {
        // DriveWorks context başlatılıyor
        dwContextParameters sdk_params = {};
        CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));
        CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));

        // Sensör parametreleri hazırlanıyor
        dwSensorParams params;
        std::string parameter_string = "output-format=yuv,fifo-size=3";
        parameter_string += ",camera-type=" + args_["camera-type"].as<std::string>();
        parameter_string += ",csi-port=" + args_["camera-port"].as<std::string>();
        parameter_string += ",slave=" + args_["tegra-slave"].as<std::string>();

        if (args_["custom-board"].as<std::string>() == "1") {
            parameter_string += ",custom-board=1";
            params.auxiliarydata = args_["custom-config"].as<std::string>().c_str();
        }
        params.parameters = parameter_string.c_str();
        params.protocol = "camera.gmsl";

        CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
        CHECK_DW_ERROR(dwSensor_start(camera_));

        // Sensörden ilk frame alınarak sensör başlatılıyor
        dwCameraFrameHandle_t frame;
        dwStatus status = DW_NOT_READY;
        do {
            status = dwSensorCamera_readFrame(&frame, 0, 66000, camera_);
        } while(status == DW_NOT_READY);
        if(status != DW_SUCCESS) {
            throw std::runtime_error("Cameras did not start correctly");
        }
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));

        // Sensör özellikleri alınıyor
        CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
        ROS_INFO("Sensor resolution: %dx%d at %f FPS", 
                 camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);

        // ROS başlatılıyor
        ros::NodeHandle n;
        gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw_cg2Open", 1);
        ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();

        // Native NVMedia görüntü, sensörün orijinal çözünürlüğüyle (örneğin 1920x1208) oluşturuluyor
        dwImageProperties native_prop{};
        native_prop.width  = camera_properties_.resolution.x;
        native_prop.height = camera_properties_.resolution.y;
        native_prop.type   = DW_IMAGE_NVMEDIA;
        native_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
        CHECK_DW_ERROR(dwImage_create(&frame_rgb_native_, native_prop, sdk_));
        ROS_INFO("Native NVMedia image created: %dx%d", native_prop.width, native_prop.height);
    }

    ~CameraGMSL() {
        ROS_INFO("Destructor called, cleaning up...");
        if(camera_) {
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
        }
        if(frame_rgb_native_) { dwImage_destroy(&frame_rgb_native_); }
        dwSAL_release(&sal_);
        dwRelease(&sdk_);
        dwLogger_release();
    }

    void publish() {
        ros::Rate loop_rate(15);
        int count = 0;
        while(ros::ok()) {
            try {
                dwCameraFrameHandle_t frame;
                uint32_t camera_sibling_id = 0;
                dwImageHandle_t frame_yuv = DW_NULL_HANDLE;

                // Kameradan frame alınıyor
                CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, camera_sibling_id, 132000, camera_));

                // YUV formatındaki görüntü NVMedia üzerinden alınıyor
                dwImageNvMedia* nvmedia_yuv_img_ptr = nullptr;
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));

                // frame_yuv, NVMedia imajına bağlanıyor
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));

                // --- AŞAMA 1: YUV görüntü native (orijinal) çözünürlükte RGBA8 formatına dönüştürülüyor ---
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_native_, frame_yuv, sdk_));

                // --- AŞAMA 2: OpenCV kullanılarak native görüntü 640x480 boyutuna yeniden ölçeklendiriliyor ---
                dwImageNvMedia* nvmedia_rgb_native = nullptr;
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_native, frame_rgb_native_));

                sensor_msgs::Image &img_msg = *ros_img_ptr_;
                std_msgs::Header header;
                header.seq = count;
                header.stamp = ros::Time::now();
                img_msg.header = header;

                // NVMedia görüntü kilitleniyor
                NvMediaImageSurfaceMap surfaceMap;
                if(NvMediaImageLock(nvmedia_rgb_native->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK) {
                    int nativeWidth = nvmedia_rgb_native->prop.width;
                    int nativeHeight = nvmedia_rgb_native->prop.height;

                    // OpenCV Mat oluşturuluyor (NVMedia görüntü verisinin sürekli bellek düzeni varsayılmıştır)
                    cv::Mat nativeImage(nativeHeight, nativeWidth, CV_8UC4, surfaceMap.surface[0].mapping);
                    cv::Mat resizedImage;
                    cv::resize(nativeImage, resizedImage, cv::Size(640, 480));

                    img_msg.height = resizedImage.rows;
                    img_msg.width  = resizedImage.cols;
                    img_msg.encoding = sensor_msgs::image_encodings::RGBA8;
                    img_msg.step = resizedImage.cols * resizedImage.elemSize();
                    size_t img_size = img_msg.step * img_msg.height;
                    img_msg.data.resize(img_size);
                    memcpy(&img_msg.data[0], resizedImage.data, img_size);

                    gmsl_pub_img_.publish(ros_img_ptr_);
                    NvMediaImageUnlock(nvmedia_rgb_native->img);
                }

                // Temizlik
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv));
                CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));

                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            } catch(const std::runtime_error &e) {
                ROS_ERROR("Runtime error: %s", e.what());
            }
        }
    }
};

int main(int argc, char **argv) {
    try {
        po::options_description desc{"Options"};
        desc.add_options()
            ("help,h", "Help screen")
            ("camera-type", po::value<std::string>()->default_value("ar0231-rccb-bae-sf3324"), "Camera GMSL type")
            ("camera-port", po::value<std::string>()->default_value("a"), "Camera CSI port")
            ("tegra-slave", po::value<std::string>()->default_value("0"), "Tegra slave mode")
            ("custom-board", po::value<std::string>()->default_value("0"), "Custom board flag")
            ("custom-config", po::value<std::string>()->default_value(""), "Custom board configuration");

        po::variables_map args;
        po::store(parse_command_line(argc, argv, desc), args);
        po::notify(args);

        ros::init(argc, argv, "camera_gmsl");

        CameraGMSL cam(args);
        cam.publish();
    } catch(const std::runtime_error &e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
    return 0;
}
