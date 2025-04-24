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
void initCameraDebug(const std::string& camType, int csiPort, bool isSlave) {
    std::ostringstream oss;
    oss << "output-format=processed,fifo-size=3,camera-type=" << camType;
    oss << ",csi-port="  << csiPort;
    oss << ",slave="     << (isSlave ? "1" : "0");
    std::string paramsStr = oss.str();

    std::cout << "[DEBUG] GMSL params = " << paramsStr << std::endl;
    // Keep node alive to view output
    ros::Rate rate(1);
    while (ros::ok()) {
        rate.sleep();
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
