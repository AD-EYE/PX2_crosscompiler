// --- main.cpp: PX2 + DriveWorks 1.2 — Debug Build to Print GMSL Params — ROS Node

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <signal.h>

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>        // DW_VERSION
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>               // dwImage_create, dwImage_copyConvert, dwImage_getCPU, dwImage_destroy

#include <stdexcept>
#include <string>

//-----------------------------------------
// Error-check macro
//-----------------------------------------
#define CHECK_DW_ERROR(expr) do {                         \
    dwStatus _status = (expr);                            \
    if (_status != DW_SUCCESS) {                          \
        std::cerr << "DriveWorks error " << _status      \
                  << " at " << __FILE__ << ":"         \
                  << __LINE__ << std::endl;               \
        throw std::runtime_error(std::to_string(_status));\
    }                                                     \
} while(0)

//-----------------------------------------
// Init SDK & HAL
//-----------------------------------------
void initDriveWorks() {
    dwContextParameters params = {};
    dwContextHandle_t sdk = nullptr;
    CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &params));
    dwSALHandle_t sal = nullptr;
    CHECK_DW_ERROR(dwSAL_initialize(&sal, sdk));
    std::cout << "[DEBUG] DriveWorks SDK & SAL initialized" << std::endl;
}

//-----------------------------------------
// Debug: Print GMSL parameters only
//-----------------------------------------
// Debug: Print GMSL parameters only
//-----------------------------------------
//-----------------------------------------
// Initialize GMSL camera with refined parameters
//-----------------------------------------
void initCamera(const std::string& camType, int csiPort, bool isSlave) {
    // Build parameter string matching DriveWorks 1.2 GMSL plugin requirements
    std::ostringstream oss;
    oss << "sensor-type=gmsl,";
    oss << "camera-type=" << camType << ",";
    oss << "csi-port="  << csiPort << ",";
    oss << "pixel-format=bayer,bit-depth=10,mode=RAW10,";
    oss << "output-format=processed,fifo-count=3,";
    oss << "sensor-count=1,sensor-group=a,";
    oss << "slave="     << (isSlave ? "1" : "0");
    std::string paramsStr = oss.str();
    ROS_INFO("Using GMSL params: %s", paramsStr.c_str());

    dwSensorParams sParams = {};
    sParams.protocol   = "camera.gmsl";
    sParams.parameters = paramsStr.c_str();

    CHECK_DW_ERROR(dwSAL_createSensor(&camera_, sParams, sal_));
    CHECK_DW_ERROR(dwSensor_start(camera_));

    // Wait for first frame
    dwCameraFrameHandle_t frame;
    dwStatus st;
    do {
        st = dwSensorCamera_readFrame(&frame, 0, 100000, camera_);
    } while (st == DW_NOT_READY);
    if (st != DW_SUCCESS)
        throw std::runtime_error("Camera failed to start");

    // Log sensor properties
    dwCameraProperties props;
    CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&props, camera_));
    ROS_INFO("Camera running: %dx%d @ %.2f FPS",
             props.resolution.x, props.resolution.y, props.framerate);
    CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
}
}

//-----------------------------------------
// Entry point (debug)
//-----------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "gmsl_param_debug_node");
    try {
        initDriveWorks();
        initCameraDebug("AR0234", 0, false);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
