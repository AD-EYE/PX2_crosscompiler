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
#include <dw/image/FormatConverter.h> // Use FormatConverter

// nvmedia for surface map
#include <nvmedia_2d.h> // NvMedia 2D header
#include <nvmedia_image.h>

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
// Hedef çözünürlüğü buradan ayarlayabilir veya komut satırı argümanı yapabilirsiniz
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
    dwImageHandle_t frame_rgba_fullres_ = DW_NULL_HANDLE; // Netlik için yeniden adlandırıldı
    dwSensorHandle_t camera_ = DW_NULL_HANDLE;
    dwImageProperties camera_image_properties_; // Orijinal özellikler
    dwCameraProperties camera_properties_;

    // OpenCV GPU Mats (bir kez ayrılır)
    cv::gpu::GpuMat gpu_rgba_fullres_;
    cv::gpu::GpuMat gpu_rgba_resized_;
    cv::Mat cpu_rgba_resized_;         // Sonucu CPU'ya indirmek için
    cv::Mat cpu_rgba_fullres_header_;  // Yalnızca başlık, NvMedia tamponunu eşlemek için

    po::variables_map args_;

public:
    CameraGMSL(const po::variables_map args): args_(args)
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            dwContextParameters sdk_params = {};
            CHECK_DW_ERROR(dwInitialize(&sdk_, DW_VERSION, &sdk_params));
            CHECK_DW_ERROR(dwSAL_initialize(&sal_, sdk_));
        }

        //------------------------------------------------------------------------------
        // initializes camera
        //------------------------------------------------------------------------------
        {
            dwSensorParams params;
            std::string parameter_string = std::string("output-format=yuv,fifo-size=3"); // Kameradan YUV çıktısını koru

            parameter_string             += std::string(",camera-type=") + args_["camera-type"].as<std::string>().c_str();
            parameter_string             += std::string(",csi-port=") + args_["camera-port"].as<std::string>().c_str();
            parameter_string             += std::string(",slave=") + args_["tegra-slave"].as<std::string>().c_str();

            if (args_["custom-board"].as<std::string>().compare("1") == 0)
            {
                parameter_string             += ",custom-board=1";
                params.auxiliarydata           = args_["custom-config"].as<std::string>().c_str(); // Doğru anahtarı kullan
            }

            params.parameters           = parameter_string.c_str();
            params.protocol             = "camera.gmsl";

            CHECK_DW_ERROR(dwSAL_createSensor(&camera_, params, sal_));
            CHECK_DW_ERROR(dwSensor_start(camera_));

            // İlk kareyi bekle
            dwCameraFrameHandle_t frame_handle; // dwImageHandle_t'den farklı bir isim kullanın
            dwStatus status = DW_NOT_READY;
            int tries = 5; // Deneme sayısını sınırla
             while (status == DW_NOT_READY && tries-- > 0) {
                 status = dwSensorCamera_readFrame(&frame_handle, 0, 100000, camera_); // Zaman aşımını biraz artır
                 if (status == DW_SUCCESS) {
                     CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));
                 } else if (status != DW_NOT_READY) {
                     // Diğer hata
                     break;
                 } else {
                      ros::Duration(0.1).sleep(); // Hazır değilse kısa uyku
                 }
            }

            if (status != DW_SUCCESS) {
                 throw std::runtime_error("Kamera doğru şekilde başlamadı veya zaman aşımına uğradı. Son durum: " + std::string(dwGetStatusName(status)));
            }

            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
            ROS_INFO("Kamera başarıyla başlatıldı: çözünürlük %dx%d, kare hızı %f FPS",
                camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);

            // Orijinal özellikleri sakla (camera_properties_ zaten tutuyor olsa da)
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&camera_image_properties_, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_));
             ROS_INFO("Kamera NATIVE_PROCESSED özellikleri: type %d, format %d",
                 camera_image_properties_.type, camera_image_properties_.format); // NVMEDIA, YUV* olmalı

        }

         // ROS initialization
        {
            ros::VP_string ros_str; // Potansiyel olarak eski ROS sürümleri için gerekli
             ros::init(ros_str, "camera_gmsl", ros::init_options::NoSigintHandler); // Daha basit init

            ros::NodeHandle n;
            // Yeniden boyutlandırılmış görüntüyü yayınla
            gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_resized", 1);

            ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
            ROS_INFO("ROS başarıyla başlatıldı");
        }

        // Nvmedia and OpenCV GPU initialization
        {
            // TAM Çözünürlükteki RGBA için DW Görüntüsü oluştur (GPU üzerinde)
            dwImageProperties rgba_img_prop = {};
            rgba_img_prop.width = camera_properties_.resolution.x;
            rgba_img_prop.height = camera_properties_.resolution.y;
            rgba_img_prop.type = DW_IMAGE_NVMEDIA;          // Hedef NvMedia (GPU tamponu için)
            rgba_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8; // Hedef RGBA
            // rgba_img_prop.memoryLayout = DW_IMAGE_MEMORY_LAYOUT_PITCH; // BU SATIRI KALDIR - DW 1.2'de mevcut değil

            CHECK_DW_ERROR(dwImage_create(&frame_rgba_fullres_, rgba_img_prop, sdk_));
            ROS_INFO("Tam çözünürlüklü RGBA NvMedia dwImage başarıyla başlatıldı (%dx%d)",
                     rgba_img_prop.width, rgba_img_prop.height);

            // OpenCV GPU Mat'larını önceden ayır
            // Not: OpenCV 2.4'te genellikle bunları doğru boyutta create() ile oluşturmak gerekir
            gpu_rgba_fullres_.create(camera_properties_.resolution.y, camera_properties_.resolution.x, CV_8UC4);
            gpu_rgba_resized_.create(TARGET_HEIGHT, TARGET_WIDTH, CV_8UC4);

             // GPU aygıtının kullanılabilir olup olmadığını kontrol et (isteğe bağlı sağlık kontrolü)
             if (cv::gpu::getCudaEnabledDeviceCount() == 0) {
                 throw std::runtime_error("OpenCV için CUDA özellikli GPU aygıtı bulunamadı.");
             }
             // cv::gpu::setDevice(0); // Gerekirse aygıtı açıkça ayarla

             ROS_INFO("Yeniden boyutlandırma için OpenCV GPU Mat'ları başarıyla başlatıldı (%dx%d -> %dx%d)",
                      camera_properties_.resolution.x, camera_properties_.resolution.y,
                      TARGET_WIDTH, TARGET_HEIGHT);
        }
    }

    ~CameraGMSL()
    {
        ROS_INFO("Destructor: Kaynaklar temizleniyor...");
        // ROS burada başlatıldıysa düzgün kapatıldığından emin ol
        if (ros::isInitialized()) {
             ros::shutdown();
        }

        if (camera_ != DW_NULL_HANDLE) { // Yok etmeden önce handle kontrolü
            dwSensor_stop(camera_);
            dwSAL_releaseSensor(&camera_);
            camera_ = DW_NULL_HANDLE; // Çift serbest bırakmayı önle
        }

        // Oluşturulan görüntüyü yok et
        if (frame_rgba_fullres_ != DW_NULL_HANDLE) { // Yok etmeden önce handle kontrolü
            dwImage_destroy(&frame_rgba_fullres_);
            frame_rgba_fullres_ = DW_NULL_HANDLE;
        }

        // Release OpenCV GPU Mats (otomatik olarak destructors tarafından halledilir)

        if (sal_ != DW_NULL_HANDLE) { // Serbest bırakmadan önce handle kontrolü
            dwSAL_release(&sal_);
            sal_ = DW_NULL_HANDLE;
        }
        if (sdk_ != DW_NULL_HANDLE) { // Serbest bırakmadan önce handle kontrolü
            dwRelease(&sdk_);
            sdk_ = DW_NULL_HANDLE;
        }

        // dwLogger_release(); // Yalnızca açıkça başlatıldıysa
        ROS_INFO("Temizlik tamamlandı.");
    }

    void publish()
    {
        std::string cam_type = args_["camera-type"].as<std::string>();
        ROS_INFO("Kamera tipi: %s", cam_type.c_str());
        ROS_INFO("Yayınlama için hedef çözünürlük: %dx%d", TARGET_WIDTH, TARGET_HEIGHT);
        ROS_INFO("Görüntüler yayınlanmaya başlıyor...");

        ros::Rate loop_rate(camera_properties_.framerate > 0 ? camera_properties_.framerate : 15); // Varsa kamera kare hızını kullan
        uint32_t count = 0;

        dwImageHandle_t frame_yuv = DW_NULL_HANDLE; // YUV çerçeve sarmalayıcısı için handle

        try
        {
            while (ros::ok())
            {
                dwTime_t timeout = 1000000 / (camera_properties_.framerate > 0 ? camera_properties_.framerate : 15) + 10000; // Kare hızına göre zaman aşımı
                dwCameraFrameHandle_t frame_handle; // dwImageHandle_t'den farklı bir isim kullanın
                dwImageNvMedia* nvmedia_yuv_img_ptr = nullptr;
                dwImageNvMedia* nvmedia_rgba_fullres_img_ptr = nullptr;

                // 1. Kameradan kare oku (YUV NvMedia tamponu)
                dwStatus read_status = dwSensorCamera_readFrame(&frame_handle, 0, timeout, camera_);
                if (read_status == DW_TIME_OUT) {
                    ROS_WARN_THROTTLE(1.0, "Kamera karesi okunurken zaman aşımı");
                    continue; // Bu iterasyonu atla
                }
                CHECK_DW_ERROR(read_status); // Diğer hataları işle

                // 2. Ham YUV verisi için NvMedia handle al
                // DW_CAMERA_OUTPUT_NATIVE_PROCESSED kullanın, çünkü genellikle dönüştürücüler için gerekli girdidir
                CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame_handle));

                 // NvMediaImage'i copyConvert için bir dwImageHandle içine sarmalamak gerekiyor
                 // Önceki iterasyon hatasından frame_yuv varsa yok et
                 if (frame_yuv != DW_NULL_HANDLE) {
                     dwImage_destroy(&frame_yuv);
                     frame_yuv = DW_NULL_HANDLE;
                 }
                CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));

                // 3. YUV (GPU) -> RGBA (GPU) tam çözünürlükte dönüştür
                CHECK_DW_ERROR(dwImage_copyConvert(frame_rgba_fullres_, frame_yuv, sdk_));

                // Bu kare için artık YUV sarmalayıcı handle'a ihtiyacımız yok
                CHECK_DW_ERROR(dwImage_destroy(&frame_yuv)); // YUV sarmalayıcısını yok et
                frame_yuv = DW_NULL_HANDLE;                 // Handle'ı sıfırla

                 // --- OpenCV GPU Yeniden Boyutlandırma BAŞLANGIÇ ---

                // 4. Tam çözünürlüklü RGBA görüntüsü için NvMedia işaretçisini al
                CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgba_fullres_img_ptr, frame_rgba_fullres_));

                // 5. NvMedia RGBA tamponunu CPU tarafından erişilebilir olacak şekilde eşle (ancak muhtemelen hala GPU sabitlenmiş bellekte)
                NvMediaImageSurfaceMap surfaceMap;
                if (NvMediaImageLock(nvmedia_rgba_fullres_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
                {
                    // 6. Eşlenmiş verilere işaret eden bir OpenCV CPU Mat başlığı oluştur (henüz derin kopya YOK)
                    // NvMedia yüzeyinden pitch kullanın (genişlik*bpp'den farklıysa)
                     size_t full_res_stride = surfaceMap.surface[0].pitch; // Doğru adım için NvMedia'dan pitch kullanın
                     // Geçici bir Mat başlığı oluştur
                     cv::Mat temp_cpu_header(nvmedia_rgba_fullres_img_ptr->prop.height,
                                             nvmedia_rgba_fullres_img_ptr->prop.width,
                                             CV_8UC4, // RGBA formatı
                                             surfaceMap.surface[0].mapping,
                                             full_res_stride);

                     // 7. CPU tarafından erişilebilir eşlenmiş tampondan veriyi OpenCV GpuMat'a yükle
                     // Bu bir kopya işlemidir (potansiyel olarak Host->Device DMA)
                     gpu_rgba_fullres_.upload(temp_cpu_header); // Yükleme için geçici başlığı kullan

                     // 8. Yükleme biter bitmez NvMedia yüzeyinin kilidini aç
                     NvMediaImageUnlock(nvmedia_rgba_fullres_img_ptr->img);

                     // 9. Yeniden boyutlandırma işlemini tamamen GPU üzerinde gerçekleştir
                     cv::gpu::resize(gpu_rgba_fullres_, gpu_rgba_resized_, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_LINEAR);

                     // 10. Yeniden boyutlandırılmış görüntüyü GPU'dan CPU Mat'ına geri indir
                     // Bu bir kopya işlemidir (Device->Host)
                     gpu_rgba_resized_.download(cpu_rgba_resized_); // Sınıf üyesi cpu_rgba_resized_'a indir

                     // --- OpenCV GPU Yeniden Boyutlandırma SON ---


                     // --- ROS Mesajını YENİDEN BOYUTLANDIRILMIŞ veriyle doldur ---
                    std_msgs::Header header; // Döngü içinde başlık oluştur
                    header.seq = count++;
                    header.stamp = ros::Time::now(); // Geçerli zamanı kullan
                    header.frame_id = "camera_link"; // İlgili bir frame_id ata

                    sensor_msgs::Image &img_msg = *ros_img_ptr_; // Sınıf üyesi işaretçiyi kullan
                    img_msg.header = header;
                    img_msg.height = TARGET_HEIGHT; // Hedef yüksekliği kullan
                    img_msg.width = TARGET_WIDTH;   // Hedef genişliği kullan
                    img_msg.encoding = sensor_msgs::image_encodings::RGBA8; // Kodlama CV_8UC4 ve DW formatıyla eşleşir
                    img_msg.is_bigendian = 0;
                    img_msg.step = cpu_rgba_resized_.step; // Adımı indirilen *yeniden boyutlandırılmış* CPU mat'ından al

                    size_t resized_img_size = img_msg.step * img_msg.height;
                    img_msg.data.resize(resized_img_size);

                    // 11. Yeniden boyutlandırılmış CPU mat'ından veriyi ROS mesaj tamponuna kopyala
                    memcpy(&img_msg.data[0], cpu_rgba_resized_.data, resized_img_size);

                    // 12. Yeniden boyutlandırılmış görüntüyü içeren ROS mesajını yayınla
                    gmsl_pub_img_.publish(ros_img_ptr_);

                } else {
                    ROS_ERROR("Okuma için NvMedia görüntü tamponu kilitlenemedi.");
                    // Kilit başarısız olduysa kilidi açmaya çalışma
                }

                // 13. Orijinal kamera kare tamponunu iade et
                CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame_handle));

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        catch (const std::runtime_error &e)
        {
             ROS_ERROR("Yayınlama döngüsünde çalışma zamanı hatası: %s", e.what());
             // Çıkmadan veya yeniden fırlatmadan önce gerekli temizliği yap
             if (frame_yuv != DW_NULL_HANDLE) {
                 dwImage_destroy(&frame_yuv); // Döngü ortasında hata oluştuysa geçici handle'ı temizle
             }
             // Ciddiyete göre yeniden fırlatmayı veya çıkmayı düşün
             // throw; // main'in yakalamasını istiyorsanız yeniden fırlatın
        }
        catch (const std::exception& e) { // Daha genel std::exception yakala
             ROS_ERROR("Yayınlama döngüsünde standart istisna yakalandı: %s", e.what());
             if (frame_yuv != DW_NULL_HANDLE) {
                 dwImage_destroy(&frame_yuv);
             }
             // throw;
        }
        catch (...) {
             ROS_ERROR("Yayınlama döngüsünde bilinmeyen istisna yakalandı.");
             if (frame_yuv != DW_NULL_HANDLE) {
                 dwImage_destroy(&frame_yuv);
             }
             // throw;
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, char *argv[]) // Standart main imzası kullan
{
    po::options_description desc{"Seçenekler"};
    desc.add_options()
        ("help,h", "Yardım ekranı")
        ("camera-type", po::value<std::string>()->default_value("ar0231-rccb-bae-sf3324"),
            "Kamera GMSL tipi (bkz. sample_sensors_info)")
        ("camera-port", po::value<std::string>()->default_value("a"), "Kamera CSI portu (a, c, e, g)")
        ("tegra-slave", po::value<std::string>()->default_value("0"), "Slave modunu etkinleştir (yalnızca Tegra B)")
        ("custom-board", po::value<std::string>()->default_value("0"), "Özel kart yapılandırması kullan (doğruysa 1 yapın)")
        ("custom-config", po::value<std::string>()->default_value(""), "Özel kart/kamera yapılandırma dosyası/verisi yolu (custom-board=1 ise kullanılır)");
        // İstenirse hedef genişlik/yükseklik seçenekleri ekleyin
        // ("target-width", po::value<int>()->default_value(640), "Çıktı görüntüsü için hedef genişlik")
        // ("target-height", po::value<int>()->default_value(480), "Çıktı görüntüsü için hedef yükseklik")


    po::variables_map args;
    try {
        po::store(po::parse_command_line(argc, argv, desc), args);

        if (args.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(args); // Hataları kontrol et, varsayılan değerleri uygula

        // Komut satırından sağlanmışsa hedef çözünürlüğü güncelle
        // if (args.count("target-width")) TARGET_WIDTH = args["target-width"].as<int>();
        // if (args.count("target-height")) TARGET_HEIGHT = args["target-height"].as<int>();

    } catch (const po::error &ex) {
        std::cerr << "Komut satırı seçenekleri ayrıştırılırken hata: " << ex.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    } catch (const std::exception& e) {
         std::cerr << "Hata: " << e.what() << std::endl;
         return 1;
    }


    try {
        ROS_INFO("CameraGMSL düğümü başlatılıyor...");
        // ROS'u main içinde başlatmak daha standarttır
        // ros::init(argc, argv, "camera_gmsl_publisher", ros::init_options::NoSigintHandler); // Eğer sinyal yönetimi gerekiyorsa
        // Eğer CameraGMSL içinde başlatılıyorsa bu satırı kaldırın.

        CameraGMSL cam(args);
        cam.publish();
    } catch (const std::runtime_error &e) {
        ROS_FATAL("Başlatma veya işleme başarısız oldu: %s", e.what());
        std::cerr << "KRİTİK HATA: " << e.what() << std::endl; // cerr'e de yazdır
        return 1; // Başarısızlığı belirt
    }
     catch (const std::exception& e) {
         ROS_FATAL("Standart istisna yakalandı: %s", e.what());
         std::cerr << "KRİTİK HATA (std::exception): " << e.what() << std::endl;
         return 1;
    }
    catch (...) {
         ROS_FATAL("Yürütme sırasında bilinmeyen istisna yakalandı.");
         std::cerr << "KRİTİK HATA (bilinmeyen istisna)" << std::endl;
         return 1;
    }

    ROS_INFO("CameraGMSL düğümü tamamlandı.");
    return 0; // Başarıyı belirt
}
