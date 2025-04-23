// --- main.cpp: PX2 + DriveWorks 1.2 — Debug Build to Print GMSL Params — ROS Node

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>        // DW_VERSION
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>

#include <dw/image/Image.h>               // dwImage_create, dwImage_copyConvert, dwImage_getCPU, dwImage_destroy

#include <stdexcept>
#include <string>
#include <sstream>
#include <signal.h>

//-----------------------------------------
// Error-check macro
//-----------------------------------------
#define CHECK_DW_ERROR(expr) do {                         \
    dwStatus _status = (expr);                            \
    if (_status != DW_SUCCESS) {                          \
        ROS_ERROR("DriveWorks error %d at %s:%d",       \
                  _status, __FILE__, __LINE__);           \
        throw std::runtime_error(std::to_string(_status));\
    }                                                     \
} while(0)

//-----------------------------------------
// Globals
//-----------------------------------------
static dwContextHandle_t sdk_    = DW_NULL_HANDLE;
static dwSALHandle_t     sal_    = DW_NULL_HANDLE;


//-----------------------------------------
// Init SDK & HAL
//-----------------------------------------
void initDriveWorks() {
    dwContextParameters params = {};
    CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &params));
    CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
    ROS_INFO("DriveWorks SDK & SAL initialized");
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

    ROS_INFO("[DEBUG] GMSL params = %s", paramsStr.c_str());
    // Return early: do not create sensor yet
    return;
}

//-----------------------------------------
// Entry point (debug)
//-----------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "gmsl_param_debug_node");
    ros::NodeHandle nh;
    try {
        initDriveWorks();
        initCameraDebug("AR0234", 0, false);
    } catch (const std::exception& e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }
    return 0;
}
