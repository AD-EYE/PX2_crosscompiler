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
#include <dw/image/FormatConverter.h> // Use FormatConverter instead of vibrante specific one if available/needed

// nvmedia for surface map
#include <nvmedia_2d.h> // Include NvMedia 2D header
#include <nvmedia_image.h>
// #include "nvmedia_ijpe.h" // IJPE likely not needed for this
// #include "nvmedia_surface.h" // NvMediaImageSurfaceMap is in nvmedia_image.h or nvmedia_2d.h

// Renderer (Keep if needed elsewhere, not strictly required for this modification)
// #include <dw/renderer/Renderer.h>

#include <sstream>
#include <boost/program_options.hpp>

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp> // OpenCV 2.4 GPU module

#define CHECK_DW_ERROR(x) { \
                    dwStatus result = x; \
                    if(result!=DW_SUCCESS) { \
                        throw std::runtime_error(std::string("DW Error ") \
                                                + dwGetStatusName(result) \
                                                + std::string(" executing DW function:\n " #x) \
                                                + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
                    }};

// --- Define Target Resolution ---
// You can make these command-line arguments or constants
const int TARGET_WIDTH = 640;
const int TARGET_HEIGHT = 480;
// -----------------------------

namespace po = boost::program_options;

class CameraGMSL
{
private:
    // Driveworks Context and SAL
    dwContextHandle_t sdk_                  = DW_NULL_HANDLE;
    dwSALHandle_t sal_                      = DW_NULL_HANDLE;

    // ROS variables
    ros::Publisher gmsl_pub_img_;
    sensor_msgs::ImagePtr ros_img_ptr_;

    // Image handles and properties
    dwImageHandle_t frame_rgba_fullres_ = DW_NULL_HANDLE; // Renamed for clarity
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwImageProperties camera_image_properties_; // Original properties
    dwCameraProperties camera_properties_;

    // OpenCV GPU Mats (allocated once)
    cv::gpu::GpuMat gpu_rgba_fullres_;
    cv::gpu::GpuMat gpu_rgba_resized_;
    cv::Mat cpu_rgba_resized_;         // For downloading result to CPU
    cv::Mat cpu_rgba_fullres_header_;  // Header only, to map NvMedia buffer

    po::variables_map args_;

public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            dwContextParameters sdk_params = {};
            // Consider enabling CUDA context for potential interop if needed later
            // sdk_params.cudaDeviceIndex = 0; // Or appropriate device
            CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));
            CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
        }

        //------------------------------------------------------------------------------
        // initializes camera
        //------------------------------------------------------------------------------
        {
            dwSensorParams params;
            std::string parameter_string = std::string("output-format=yuv,fifo-size=3"); // Keep YUV output from camera

            parameter_string             += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string             += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string             += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

            if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                parameter_string             += ",custom-board=1";
                params.auxiliarydata           = args_["custom-config"].as<std::string>().c_str(); // Use correct key
            }

            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
            CHECK_DW_ERROR(dwSensor_start(camera_));

            // Wait for first frame
            dwCameraFrameHandle_t frame;
            dwStatus status = DW_NOT_READY;
            int tries = 5; // Limit tries
             while (status == DW_NOT_READY && tries-- > 0) {
                 status = dwSensorCamera_readFrame(&frame, 0, 100000, camera_); // Increased timeout slightly
                 if (status == DW_SUCCESS) {
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
                 } else if (status != DW_NOT_READY) {
                     // Other error
                     break;
                 } else {
                      ros::Duration(0.1).sleep(); // Short sleep if not ready
                 }
            }

            if (status != DW_SUCCESS) {
                 throw std::runtime_error("Camera did not start correctly or timed out. Last status: " + std::string(dwGetStatusName(status)));
            }

            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Successfully initialized camera with resolution %dx%d at framerate %f FPS",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);

            // Store original properties (although camera_properties_ holds them)
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&camera_image_properties_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_));
             ROS_INFO("Camera NATIVE_PROCESSED properties: type %d, format %d",
                 camera_image_properties_.type, camera_image_properties_.format); // Should be NVMEDIA, YUV*

        }

         // ROS initialization
        {
            ros::VP_string ros_str; // Needed for older ROS versions potentially
            // ros::init(ros_str, "camera_gmsl"); // Use this if argc/argv aren't easily available
            // Temporarily remove argc, argv from init if causing issues, but standard is:
            // int ros_argc = 0; char** ros_argv = nullptr; // If you don't have main's args easily
            // ros::init(ros_argc, ros_argv, "camera_gmsl", ros::init_options::NoSigintHandler); // Or handle signals properly
             ros::init(ros_str, "camera_gmsl", ros::init_options::NoSigintHandler); // Simpler init

            ros::NodeHandle n;
            // Publish resized image
            gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_resized", 1);

            ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
            ROS_INFO("Successfully initialized ROS");
        }

        // Nvmedia and OpenCV GPU initialization
        {
            // Create DW Image for FULL Resolution RGBA (on GPU)
            dwImageProperties rgba_img_prop = {};
            rgba_img_prop.width = camera_properties_.resolution.x;
            rgba_img_prop.height = camera_properties_.resolution.y;
            rgba_img_prop.type = DW_IMAGE_NVMEDIA;          // Target NvMedia for GPU buffer
            rgba_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8; // Target RGBA
            rgba_img_prop.memoryLayout = DW_IMAGE_MEMORY_LAYOUT_PITCH; // Default layout
            CHECK_DW_ERROR(dwImage_create(&frame_rgba_fullres_, rgba_img_prop, sdk_));
            ROS_INFO("Successfully initialized full-res RGBA NvMedia dwImage (%dx%d)",
                     rgba_img_prop.width, rgba_img_prop.height);

            // Pre-allocate OpenCV GPU Mats
            // Note: In OpenCV 2.4, you often needed to create them with size
            gpu_rgba_fullres_.create(camera_properties_.resolution.y, camera_properties_.resolution.x, CV_8UC4);
            gpu_rgba_resized_.create(TARGET_HEIGHT, TARGET_WIDTH, CV_8UC4);

             // Check if GPU device is available (optional sanity check)
             if (cv::gpu::getCudaEnabledDeviceCount() == 0) {
                 throw std::runtime_error("No CUDA-enabled GPU device found for OpenCV.");
             }
             // cv::gpu::setDevice(0); // Explicitly set device if needed

             ROS_INFO("Successfully initialized OpenCV GPU Mats for resizing (%dx%d -> %dx%d)",
                      camera_properties_.resolution.x, camera_properties_.resolution.y,
                      TARGET_WIDTH, TARGET_HEIGHT);
        }
    }

    ~CameraGMSL()
    {
        ROS_INFO("Destructor: Cleaning up resources...");
        // Ensure ROS is properly shutdown if initialized here
        if (ros::isInitialized()) {
             ros::shutdown();
        }

        if (camera_) {
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
            camera_ = DW_NULL_HANDLE; // Prevent double release
        }

        // Destroy created image
        // Check handle before destroying
        if (frame_rgba_fullres_ != DW_NULL_HANDLE) {
            dwImage_destroy(&frame_rgba_fullres_);
            frame_rgba_fullres_ = DW_NULL_HANDLE;
        }


        // Release OpenCV GPU Mats (automatically handled by destructors)

        if (sal_ != DW_NULL_HANDLE) {
            dwSAL_release(&sal_);
            sal_ = DW_NULL_HANDLE;
        }
        if (sdk_ != DW_NULL_HANDLE) {
            dwRelease(&sdk_);
            sdk_ = DW_NULL_HANDLE;
        }

        // dwLogger_release(); // Only if explicitly initialized
        ROS_INFO("Cleanup complete.");
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Camera type: %s", cam_type.c_str());
        ROS_INFO("Target resolution for publishing: %dx%d", TARGET_WIDTH, TARGET_HEIGHT);
        ROS_INFO("Starting to publish images...");

        ros::Rate loop_rate(camera_properties_.framerate > 0 ? camera_properties_.framerate : 15); // Use camera framerate if available
        uint32_t count = 0;

        dwImageHandle_t frame_yuv = DW_NULL_HANDLE; // Handle for YUV frame wrapper

        try
        {
            while (ros::ok())
            {
                dwTime_t timeout = 1000000 / (camera_properties_.framerate > 0 ? camera_properties_.framerate : 15) + 10000; // Timeout based on framerate
                dwCameraFrameHandle_t frame_handle; // Use a different name from the dwImageHandle_t
                dwImageNvMedia* nvmedia_yuv_img_ptr = nullptr;
                dwImageNvMedia* nvmedia_rgba_fullres_img_ptr = nullptr;

                // 1. Read frame from camera (YUV NvMedia buffer)
                dwStatus read_status = dwSensorCamera_readFrame(&frame_handle, 0, timeout, camera_);
                if (read_status == DW_TIME_OUT) {
                    ROS_WARN_THROTTLE(1.0, "Timeout reading camera frame");
                    continue; // Skip this iteration
                }
                CHECK_DW_ERROR(read_status); // Handle other errors

                // 2. Get NvMedia handle for the raw YUV data
                // Use DW_CAMERA_OUTPUT_NATIVE_PROCESSED as it's often the required input for converters
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));

                 // Need to wrap the NvMediaImage in a dwImageHandle for copyConvert
                 // Destroy previous frame_yuv if it exists from a prior iteration failure
                 if (frame_yuv != DW_NULL_HANDLE) {
                     dwImage_destroy(&frame_yuv);
                     frame_yuv = DW_NULL_HANDLE;
                 }
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));

                // 3. Convert YUV (GPU) -> RGBA (GPU) at full resolution
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgba_fullres_, frame_yuv, sdk_));

                // We no longer need the YUV wrapper handle for this frame
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv)); // Destroy YUV wrapper
                frame_yuv = DW_NULL_HANDLE;                 // Reset handle

                 // --- OpenCV GPU Resizing START ---

                // 4. Get NvMedia pointer for the full-res RGBA image
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgba_fullres_img_ptr, frame_rgba_fullres_));

                // 5. Map the NvMedia RGBA buffer to be CPU-accessible (but still likely in GPU pinned memory)
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgba_fullres_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // 6. Create an OpenCV CPU Mat header pointing to the mapped data (NO deep copy yet)
                    // Use pitch from NvMedia surface if available and differs from width*bpp
                     size_t full_res_stride = surfaceMap.surface[0].pitch; // Use pitch from NvMedia for correct stride
                     cpu_rgba_fullres_header_ = cv::Mat(nvmedia_rgba_fullres_img_ptr->prop.height,
                                                      nvmedia_rgba_fullres_img_ptr->prop.width,
                                                      CV_8UC4, // RGBA format
                                                      surfaceMap.surface[0].mapping,
                                                      full_res_stride);

                     // 7. Upload data from the CPU-accessible mapped buffer to the OpenCV GpuMat
                     // This is a copy operation (potentially Host->Device DMA)
                     gpu_rgba_fullres_.upload(cpu_rgba_fullres_header_);

                     // 8. Unlock the NvMedia surface as soon as upload is done
                     NvMediaImageUnlock(nvmedia_rgba_fullres_img_ptr->img);

                     // 9. Perform resize operation entirely on the GPU
                     cv::gpu::resize(gpu_rgba_fullres_, gpu_rgba_resized_, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_LINEAR);

                     // 10. Download the resized image from GPU back to a CPU Mat
                     // This is a copy operation (Device->Host)
                     gpu_rgba_resized_.download(cpu_rgba_resized_);

                     // --- OpenCV GPU Resizing END ---


                     // --- Populate ROS Message with RESIZED data ---
                    std_msgs::Header header; // Create header inside the loop
                    header.seq = count++;
                    header.stamp = ros::Time::now(); // Use current time
                    header.frame_id = "camera_link"; // Assign a relevant frame_id

                    sensor_msgs::Image &img_msg = *ros_img_ptr_; // Use the class member pointer
                    img_msg.header = header;
                    img_msg.height = TARGET_HEIGHT; // Use target height
                    img_msg.width = TARGET_WIDTH;   // Use target width
                    img_msg.encoding = sensor_msgs::image_encodings::RGBA8; // Encoding matches CV_8UC4 and DW format
                    img_msg.is_bigendian = 0;
                    img_msg.step = cpu_rgba_resized_.step; // Get step from the downloaded *resized* CPU mat

                    size_t resized_img_size = img_msg.step * img_msg.height;
                    img_msg.data.resize(resized_img_size);

                    // 11. Copy data from the resized CPU mat into the ROS message buffer
                    memcpy(&img_msg.data[0], cpu_rgba_resized_.data, resized_img_size);

                    // 12. Publish the ROS message containing the resized image
                    gmsl_pub_img_.publish(ros_img_ptr_);

                } else {
                    ROS_ERROR("Failed to lock NvMedia image buffer for reading.");
                    // Don't attempt unlock if lock failed
                }

                // 13. Return the original camera frame buffer
                CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        catch (const std::runtime_error &e)
        {
             ROS_ERROR("Runtime error in publish loop: %s", e.what());
             // Perform necessary cleanup before exiting or rethrowing
             if (frame_yuv != DW_NULL_HANDLE) {
                 dwImage_destroy(&frame_yuv); // Clean up temp handle if error occurred mid-loop
             }
             // Consider re-throwing or exiting based on severity
             // throw; // Re-throw if you want main to catch it
        }
        catch (...) {
             ROS_ERROR("Unknown exception caught in publish loop.");
             if (frame_yuv != DW_NULL_HANDLE) {
                 dwImage_destroy(&frame_yuv);
             }
             // throw;
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, char *argv[]) // Use standard main signature
{
    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("camera-type", po::value<std::string>()->default_value("ar0231-rccb-bae-sf3324"),
            "Camera GMSL type (see sample_sensors_info)")
        ("camera-port", po::value<std::string>()->default_value("a"), "Camera CSI port (a, c, e, g)")
        ("tegra-slave", po::value<std::string>()->default_value("0"), "Enable slave mode (Tegra B only)")
        ("custom-board", po::value<std::string>()->default_value("0"), "Use custom board config (set to 1 if true)")
        ("custom-config", po::value<std::string>()->default_value(""), "Path to custom board/camera config file/data (used if custom-board=1)");
        // Add target width/height options if desired
        // ("target-width", po::value<int>()->default_value(640), "Target width for output image")
        // ("target-height", po::value<int>()->default_value(480), "Target height for output image")


    po::variables_map args;
    try {
        po::store(po::parse_command_line(argc, argv, desc), args);

        if (args.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(args); // Check for errors, apply default values

        // Update target resolution if provided via command line
        // if (args.count("target-width")) TARGET_WIDTH = args["target-width"].as<int>();
        // if (args.count("target-height")) TARGET_HEIGHT = args["target-height"].as<int>();

    } catch (const po::error &ex) {
        std::cerr << "Error parsing command line options: " << ex.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    } catch (const std::exception& e) {
         std::cerr << "Error: " << e.what() << std::endl;
         return 1;
    }


    try {
        ROS_INFO("Initializing CameraGMSL node...");
        CameraGMSL cam(args);
        cam.publish();
    } catch (const std::runtime_error &e) {
        ROS_FATAL("Initialization or processing failed: %s", e.what());
        // Consider ROS logging or specific error handling here
        std::cerr << "FATAL ERROR: " << e.what() << std::endl; // Also print to cerr
        return 1; // Indicate failure
    }
     catch (const std::exception& e) {
         ROS_FATAL("Caught standard exception: %s", e.what());
         std::cerr << "FATAL ERROR (std::exception): " << e.what() << std::endl;
         return 1;
    }
    catch (...) {
         ROS_FATAL("Caught unknown exception during execution.");
         std::cerr << "FATAL ERROR (unknown exception)" << std::endl;
         return 1;
    }


    ROS_INFO("CameraGMSL node finished.");
    return 0; // Indicate success
}
