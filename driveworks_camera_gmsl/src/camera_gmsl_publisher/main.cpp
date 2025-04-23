// --- main.cpp: PX2 + DriveWorks 1.2 — Full Example — Half-Res (960×604) + GPU Offload + ROS

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>        // for DW_VERSION
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>               // dwImage_create, dwImage_copyConvert, dwImage_getCpuPointer, dwImage_release

#include <stdexcept>
#include <string>
#include <signal.h>

//-----------------------------------------
// Error-check macro
//-----------------------------------------
#define CHECK_DW_ERROR(expr) do {                       \
    dwStatus _status = (expr);                          \
    if (_status != DW_SUCCESS) {                        \
        ROS_ERROR("DriveWorks error %d at %s:%d",     \
                  _status, __FILE__, __LINE__);         \
        throw std::runtime_error(std::to_string(_status));\
    }                                                   \
} while(0)

//-----------------------------------------
// Globals
//-----------------------------------------
static dwContextHandle_t sdk_         = DW_NULL_HANDLE;
static dwSALHandle_t     sal_         = DW_NULL_HANDLE;
static dwSensorHandle_t  camera_      = DW_NULL_HANDLE;
static dwImageHandle_t   imgCUDA_half = DW_NULL_HANDLE;
static dwImageHandle_t   imgCPU_half  = DW_NULL_HANDLE;
static ros::Publisher    pub_img;

const int HALF_WIDTH  = 960;
const int HALF_HEIGHT = 604;

//-----------------------------------------
// Init SDK & SAL
//-----------------------------------------
void initDriveWorks() {
    dwContextParameters params = {};
    CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &params));
    CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
    ROS_INFO("DriveWorks SDK & SAL initialized");
}

//-----------------------------------------
// Init Camera
//-----------------------------------------
void initCamera(const std::string& camType, int csiPort, bool isSlave) {
    std::string p = "output-format=processed,";
    p += "camera-type=" + camType + ",";
    p += "csi-port=" + std::to_string(csiPort) + ",";
    p += "slave=" + (isSlave ? "1" : "0");
    dwSensorParams sParams = {};
    sParams.parameters = p.c_str();
    CHECK_DW_ERROR(dwSAL_createSensor(&camera_, sParams, sal_));
    CHECK_DW_ERROR(dwSensor_start(camera_));

    // First frame
    dwCameraFrameHandle_t frame;
    dwStatus st;
    do { st = dwSensorCamera_readFrame(&frame, 0, 100000, camera_); }
    while (st == DW_NOT_READY);
    if (st != DW_SUCCESS) throw std::runtime_error("Camera start failed");

    dwCameraProperties props;
    CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&props, camera_));
    ROS_INFO("Camera: %dx%d @ %.2f FPS", props.resolution.x,
             props.resolution.y, props.framerate);
    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
}

//-----------------------------------------
// Init Images
//-----------------------------------------
void initHalfResImages() {
    dwImageProperties gp = {};
    gp.width  = HALF_WIDTH; gp.height = HALF_HEIGHT;
    gp.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    gp.type   = DW_IMAGE_CUDA;
    CHECK_DW_ERROR(dwImage_create(&imgCUDA_half, gp, sdk_));
    dwImageProperties cp = gp;
    cp.type = DW_IMAGE_CPU;
    CHECK_DW_ERROR(dwImage_create(&imgCPU_half, cp, sdk_));
    ROS_INFO("Half-res images: %dx%d", HALF_WIDTH, HALF_HEIGHT);
}

//-----------------------------------------
// Main loop
//-----------------------------------------
void processLoop() {
    dwCameraFrameHandle_t frame;
    while (ros::ok()) {
        CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, 0, 100000, camera_));
        // NVMedia image
        dwImageNvMedia* nvPtr = nullptr;
        CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvPtr,
                        DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));
        // NVMedia->CUDA
        CHECK_DW_ERROR(dwImage_copyConvert(imgCUDA_half,
                          (dwImageHandle_t)nvPtr, sdk_));
        // CUDA->CPU
        CHECK_DW_ERROR(dwImage_copyConvert(imgCPU_half,
                          imgCUDA_half, sdk_));
        // Publish
        uint8_t* data; size_t pitch;
        CHECK_DW_ERROR(dwImage_getCpuPointer(&data, &pitch, imgCPU_half));
        sensor_msgs::Image msg;
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = "gmsl_camera";
        msg.height          = HALF_HEIGHT;
        msg.width           = HALF_WIDTH;
        msg.encoding        = sensor_msgs::image_encodings::RGBA8;
        msg.is_bigendian    = false;
        msg.step            = pitch;
        msg.data.assign(data, data + pitch*HALF_HEIGHT);
        pub_img.publish(msg);
        // Cleanup
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
        ros::spinOnce();
    }
}

//-----------------------------------------
// Shutdown
//-----------------------------------------
void sigHandler(int) {
    if (camera_) dwSensor_stop(camera_);
    if (camera_) dwSAL_releaseSensor(&camera_);
    if (imgCUDA_half) dwImage_release(&imgCUDA_half);
    if (imgCPU_half)  dwImage_release(&imgCPU_half);
    if (sal_) dwSAL_release(&sal_);
    if (sdk_) dwRelease(&sdk_);
    ros::shutdown();
    exit(0);
}

//-----------------------------------------
// Main
//-----------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "gmsl_half_res_node");
    ros::NodeHandle nh;
    pub_img = nh.advertise<sensor_msgs::Image>("camera/image_raw", 1);
    signal(SIGINT, sigHandler);
    try {
        initDriveWorks();
        initCamera("AR0234", 0, false);
        initHalfResImages();
        processLoop();
    } catch(const std::exception& e) {
        ROS_FATAL("%s", e.what());
        sigHandler(0);
    }
    return 0;
}
