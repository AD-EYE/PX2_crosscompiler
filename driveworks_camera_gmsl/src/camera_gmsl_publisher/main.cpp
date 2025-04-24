// --- main.cpp: PX2 + DriveWorks 1.2 — Half-Res GMSL → ROS Node

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <iostream>

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>        // DW_VERSION
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>     // Camera sensor API
#include <dw/image/Image.h>
#include <dw/image/Converter.hpp>         // dwImage_copyConvert

#include <stdexcept>
#include <sstream>
#include <string>

//-----------------------------------------
// Check DriveWorks return status
//-----------------------------------------
#define CHECK_DW_ERROR(expr) do {                          \
    dwStatus status = (expr);                             \
    if (status != DW_SUCCESS) {                           \
        ROS_ERROR("DriveWorks error %d at %s:%d",      \
                  status, __FILE__, __LINE__);           \
        throw std::runtime_error(std::string("DriveWorks error ") + std::to_string(status)); \
    }                                                     \
} while(0)

//-----------------------------------------
// Globals
//-----------------------------------------
static dwContextHandle_t sdk     = DW_NULL_HANDLE;
static dwSALHandle_t     sal     = DW_NULL_HANDLE;
static dwSensorHandle_t  camera  = DW_NULL_HANDLE;
static dwImageHandle_t   imgCuda = DW_NULL_HANDLE;
static dwImageHandle_t   imgCpu  = DW_NULL_HANDLE;
static ros::Publisher    pubImage;

const int HALF_W = 960;
const int HALF_H = 604;

//-----------------------------------------
// Handle SIGINT for clean shutdown
//-----------------------------------------
void sigHandler(int) {
    if (camera)    dwSensor_stop(camera);
    if (camera)    dwSAL_releaseSensor(&camera);
    if (imgCuda)   dwImage_destroy(&imgCuda);
    if (imgCpu)    dwImage_destroy(&imgCpu);
    if (sal)       dwSAL_release(&sal);
    if (sdk)       dwRelease(&sdk);
    ros::shutdown();
    exit(0);
}

//-----------------------------------------
// Entry point
//-----------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "gmsl_half_res_node");
    ros::NodeHandle nh;
    pubImage = nh.advertise<sensor_msgs::Image>("camera/image_raw", 1);
    signal(SIGINT, sigHandler);

    try {
        // Initialize DriveWorks
        dwContextParameters ctxParams = {};
        CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &ctxParams));
        CHECK_DW_ERROR(dwSAL_initialize(&sal, sdk));
        ROS_INFO("DriveWorks SDK & SAL initialized");

        // Build GMSL parameters
        std::ostringstream oss;
        oss << "camera-type=AR0234,";
        oss << "csi-port=0,";
        oss << "mode=RAW10,";
        oss << "pixel-format=bayer,";
        oss << "bit-depth=10,";
        oss << "output-format=processed,";
        oss << "fifo-size=3,";
        oss << "link=0,";
        oss << "slave=0";
        std::string paramStr = oss.str();
        ROS_INFO("[PARAMS] %s", paramStr.c_str());

        // Create and start sensor
        dwSensorParams sParams = {};
        sParams.protocol   = "camera.gmsl";
        sParams.parameters = paramStr.c_str();
        CHECK_DW_ERROR(dwSAL_createSensor(&camera, sParams, sal));
        CHECK_DW_ERROR(dwSensor_start(camera));

        // Wait for first frame
        dwCameraFrameHandle_t frame = DW_NULL_HANDLE;
        dwStatus st = DW_NOT_READY;
        while (st == DW_NOT_READY && ros::ok()) {
            st = dwSensorCamera_readFrame(&frame, 0, 100000, camera);
        }
        if (st != DW_SUCCESS) throw std::runtime_error("Camera failed to start");

        // Get properties
        dwCameraProperties props;
        CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&props, camera));
        ROS_INFO("Camera running: %dx%d @ %.2f FPS", 
                 props.resolution.x, props.resolution.y, props.framerate);
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));

        // Allocate half-resolution images
        dwImageProperties imgProp = {};
        imgProp.width  = HALF_W;
        imgProp.height = HALF_H;
        imgProp.format = DW_IMAGE_FORMAT_RGBA_UINT8;
        imgProp.type   = DW_IMAGE_CUDA;
        CHECK_DW_ERROR(dwImage_create(&imgCuda, imgProp, sdk));
        imgProp.type = DW_IMAGE_CPU;
        CHECK_DW_ERROR(dwImage_create(&imgCpu, imgProp, sdk));
        ROS_INFO("Half-res images ready: %dx%d", HALF_W, HALF_H);

        // Main loop
        ros::Rate rate(props.framerate);
        while (ros::ok()) {
            CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, 0, 100000, camera));

            // Get processed CUDA image
            dwImageHandle_t inCuda = DW_NULL_HANDLE;
            CHECK_DW_ERROR(dwSensorCamera_getImageCuda(&inCuda,
                DW_CAMERA_OUTPUT_PROCESSED, frame));

            // Downsample on GPU, convert to CPU
            CHECK_DW_ERROR(dwImage_copyConvert(imgCuda, inCuda, sdk));
            CHECK_DW_ERROR(dwImage_copyConvert(imgCpu, imgCuda, sdk));

            // Publish ROS image
            void* dataPtr = nullptr;
            size_t rowPitch = 0;
            CHECK_DW_ERROR(dwImage_getCpuPointer(&dataPtr, &rowPitch, imgCpu));
            sensor_msgs::Image msg;
            msg.header.stamp    = ros::Time::now();
            msg.header.frame_id = "gmsl_camera";
            msg.height          = HALF_H;
            msg.width           = HALF_W;
            msg.encoding        = sensor_msgs::image_encodings::RGBA8;
            msg.is_bigendian    = false;
            msg.step            = rowPitch;
            msg.data.resize(rowPitch * HALF_H);
            memcpy(msg.data.data(), dataPtr, rowPitch * HALF_H);
            pubImage.publish(msg);

            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
            ros::spinOnce();
            rate.sleep();
        }

    } catch (const std::exception& e) {
        ROS_FATAL("Fatal error: %s", e.what());
        sigHandler(0);
    }

    return 0;
}
