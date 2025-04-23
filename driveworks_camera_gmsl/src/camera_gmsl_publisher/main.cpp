// --- main.cpp: PX2 + DriveWorks 1.2 — Full Example — Half‑Res (960×604) + GPU Offload + ROS

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>        // for DW_VERSION
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>               // dwImage_copyConvert, dwImage_create, dwImage_getCpuPointer / dwImage_getCudaPointer

#include <stdexcept>
#include <string>
#include <signal.h>

//-----------------------------------------
// Error-checking macro
//-----------------------------------------
#define CHECK_DW_ERROR(expr) do {                       \
    dwStatus _status = (expr);                          \
    if (_status != DW_SUCCESS) {                        \
        const char* _msg = nullptr;                     \
        dwGetLastErrorString(&_msg);                    \
        ROS_ERROR("DriveWorks error %s at %s:%d",    \
                  _msg, __FILE__, __LINE__);           \
        throw std::runtime_error(_msg);                 \
    }                                                   \
} while(0)

//-----------------------------------------
// Global handles & params
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
// Initialize DriveWorks SDK & HAL
//-----------------------------------------
void initDriveWorks() {
    dwContextParameters sdkParams = {};
    CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdkParams));
    CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
    ROS_INFO("DriveWorks SDK & SAL initialized");
}

//-----------------------------------------
// Configure GMSL sensor for CUDA output
//-----------------------------------------
void initCamera(const std::string& camType, int csiPort, bool isSlave) {
    dwSensorParams params = {};
    params.parameters =
        std::string("output-format=processed,") +
        "camera-type="  + camType    + "," +
        "csi-port="     + std::to_string(csiPort) + "," +
        "slave="        + (isSlave?"1":"0") ;

    CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
    CHECK_DW_ERROR(dwSensor_start(camera_));

    // request CUDA output via setting output properties
    dwImageProperties outputProps = {};
    outputProps.type   = DW_IMAGE_CUDA;
    outputProps.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    CHECK_DW_ERROR(dwSensorCamera_setOutputProperties(camera_, outputProps));

    // wait for first frame
    dwCameraFrameHandle_t frame;
    dwStatus stat;
    do {
        stat = dwSensorCamera_readFrame(&frame, 0, 100000, camera_);
    } while (stat == DW_NOT_READY);
    if (stat != DW_SUCCESS) throw std::runtime_error("Camera failed to start");

    dwCameraProperties camProps;
    CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camProps, camera_));
    ROS_INFO("Camera: %dx%d @ %.2f FPS", camProps.resolution.x, camProps.resolution.y, camProps.framerate);
    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
}

//-----------------------------------------
// Create half-res CUDA & CPU images
//-----------------------------------------
void initHalfResImages() {
    dwImageProperties gpuProps = {};
    gpuProps.width  = HALF_WIDTH;
    gpuProps.height = HALF_HEIGHT;
    gpuProps.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    gpuProps.type   = DW_IMAGE_CUDA;
    CHECK_DW_ERROR(dwImage_create(&imgCUDA_half, gpuProps, sdk_));

    dwImageProperties cpuProps = gpuProps;
    cpuProps.type = DW_IMAGE_CPU;
    CHECK_DW_ERROR(dwImage_create(&imgCPU_half, cpuProps, sdk_));

    ROS_INFO("Half-res images ready: %dx%d", HALF_WIDTH, HALF_HEIGHT);
}

//-----------------------------------------
// Downsample & publish
//-----------------------------------------
void processLoop() {
    dwCameraFrameHandle_t frame;
    while (ros::ok()) {
        CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, 0, 100000, camera_));

        dwImageHandle_t inCUDA = DW_NULL_HANDLE;
        CHECK_DW_ERROR(dwSensorCamera_getImageCuda(&inCUDA, DW_CAMERA_OUTPUT_PROCESSED, frame));

        // downsample (CUDA->CUDA)
        CHECK_DW_ERROR(dwImage_copyConvert(sdk_, imgCUDA_half, inCUDA));
        // CUDA->CPU copy
        CHECK_DW_ERROR(dwImage_copyConvert(sdk_, imgCPU_half, imgCUDA_half));

        // publish to ROS
        uint8_t* cpuPtr; size_t rowPitch;
        CHECK_DW_ERROR(dwImage_getCpuPointer(&cpuPtr, &rowPitch, imgCPU_half));
        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "gmsl_camera";
        msg.height = HALF_HEIGHT;
        msg.width  = HALF_WIDTH;
        msg.encoding = sensor_msgs::image_encodings::RGBA8;
        msg.is_bigendian = false;
        msg.step = rowPitch;
        msg.data.assign(cpuPtr, cpuPtr + rowPitch * HALF_HEIGHT);
        pub_img.publish(msg);

        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
        ros::spinOnce();
    }
}

//-----------------------------------------
// Shutdown
//-----------------------------------------
void sigHandler(int sig) {
    if (camera_) dwSensor_stop(camera_);
    if (camera_) dwSAL_releaseSensor(&camera_);
    if (imgCUDA_half) dwImage_release(&imgCUDA_half);
    if (imgCPU_half) dwImage_release(&imgCPU_half);
    if (sal_) dwSAL_release(&sal_);
    if (sdk_) dwRelease(&sdk_);
    ros::shutdown();
    exit(0);
}

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
    } catch (const std::exception& e) {
        ROS_FATAL("Error: %s", e.what());
        sigHandler(0);
    }
    return 0;
}
