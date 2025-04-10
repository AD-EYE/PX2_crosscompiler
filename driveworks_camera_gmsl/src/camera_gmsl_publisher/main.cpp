CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera_properties_, camera_));
           ROS_INFO("Successfully initialized camera with resolution of %dx%d at framerate of %f FPS\n",
               camera_properties_.resolution.x, camera_properties_.resolution.y, camera_properties_.framerate);

       }

        //ROS initialization
       {
           ros::VP_string ros_str;
           ros::init(ros_str, "camera_gmsl");
           ros::NodeHandle n;
           gmsl_pub_img_ = n.advertise<sensor_msgs::Image>("camera_1/image_raw", 1);

           ros_img_ptr_ = boost::make_shared<sensor_msgs::Image>();
           ROS_INFO("Successfully initialized ros\n" );
       }

       //Nvmedia initialization
       {
           dwImageProperties rgb_img_prop{};
           rgb_img_prop.height = camera_properties_.resolution.y;
           rgb_img_prop.width = camera_properties_.resolution.x;
           rgb_img_prop.type = DW_IMAGE_NVMEDIA;
           rgb_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
           CHECK_DW_ERROR(dwImage_create(&frame_rgb_, rgb_img_prop, sdk_));
           
           // Calculate half resolution dimensions
           half_width_ = camera_properties_.resolution.x / 2;
           half_height_ = camera_properties_.resolution.y / 2;
           
           // Create half resolution image
           dwImageProperties rgb_half_img_prop{};
           rgb_half_img_prop.height = half_height_;
           rgb_half_img_prop.width = half_width_;
           rgb_half_img_prop.type = DW_IMAGE_NVMEDIA;
           rgb_half_img_prop.format = DW_IMAGE_FORMAT_RGBA_UINT8;
           CHECK_DW_ERROR(dwImage_create(&frame_rgb_half_, rgb_half_img_prop, sdk_));
           
           ROS_INFO("Successfully initialized nvmedia images. Original: %dx%d, Half: %dx%d\n", 
                    camera_properties_.resolution.x, camera_properties_.resolution.y,
                    half_width_, half_height_);
       }
       
   }

   ~CameraGMSL()
   {
       ROS_INFO("Destructor!!!");
       if (camera_) {
           dwSensor_stop(camera_);
           dwSAL_releaseSensor(&camera_);
       }

       //destroy created images
       dwImage_destroy(&frame_rgb_);
       dwImage_destroy(&frame_rgb_half_);

       dwSAL_release(&sal_);
       dwRelease(&sdk_);
       dwLogger_release();

   }

   void publish()
   {
       std::string cam_type = args_["camera-type"].as<std::string>();
       ROS_INFO("Camera type - %s \n", cam_type.c_str());
       ROS_INFO("Starting to publish images");

       try
       {
           ros::Rate loop_rate(15);
           int count = 0;
           while (ros::ok())
           {
               dwTime_t timeout = 132000; 
               dwCameraFrameHandle_t frame;
               uint32_t camera_sibling_id = 0;
               dwImageHandle_t frame_yuv;
               dwImageNvMedia* nvmedia_yuv_img_ptr;
               dwImageNvMedia* nvmedia_rgb_img_ptr;
               dwImageNvMedia* nvmedia_rgb_half_img_ptr;

               sensor_msgs::Image &img_msg = *ros_img_ptr_; // >> message to be sent
               std_msgs::Header header; // empty header
               size_t img_size;

               
               // read from camera will update the low level buffers frame of the camera
               // those frames are images with NATIVE properties that depend on the type and sensor properties set at creation
               CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, camera_sibling_id, timeout, camera_));

               CHECK_DW_ERROR(dwSensorCamera_getImageNvMedia(&nvmedia_yuv_img_ptr, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame));
               CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
               CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_half_img_ptr, frame_rgb_half_));
               CHECK_DW_ERROR(dwImage_createAndBindNvMedia(&frame_yuv, nvmedia_yuv_img_ptr->img));
               CHECK_DW_ERROR(dwImage_copyConvert(frame_rgb_, frame_yuv, sdk_));
               CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia_rgb_img_ptr, frame_rgb_));
               
               // GPU-accelerated downsampling with NvMedia
               NvMediaBlit *blit = NULL;
               blit = NvMediaBlitCreate();
               if (blit) {
                   // Configure the blit parameters for 2x downsampling
                   NvMediaRect srcRect, dstRect;
                   srcRect.x = 0;
                   srcRect.y = 0;
                   srcRect.width = nvmedia_rgb_img_ptr->prop.width;
                   srcRect.height = nvmedia_rgb_img_ptr->prop.height;
                   
                   dstRect.x = 0;
                   dstRect.y = 0;
                   dstRect.width = half_width_;
                   dstRect.height = half_height_;
                   
                   // Perform the GPU blit operation (downsampling)
                   NvMediaBlitParameters params;
                   memset(&params, 0, sizeof(params));
                   params.srcRect = &srcRect;
                   params.dstRect = &dstRect;
                   params.filterType = NVMEDIA_BLIT_FILTER_BILINEAR; // Use bilinear filtering for better quality
                   
                   NvMediaStatus status = NvMediaBlitSurface(blit, 
                                                  nvmedia_rgb_half_img_ptr->img, 
                                                  nvmedia_rgb_img_ptr->img, 
                                                  &params);
                                                  
                   if (status != NVMEDIA_STATUS_OK) {
                       ROS_WARN("GPU downsampling failed with status %d\n", status);
                   }
                   
                   NvMediaBlitDestroy(blit);
               } else {
                   ROS_WARN("Failed to create NvMediaBlit object for downsampling\n");
               }

               
               header.seq = count; // user defined counter
               header.stamp = ros::Time::now(); 
                   
               img_msg.header = header;
               img_msg.height = half_height_;
               img_msg.width = half_width_;
               img_msg.encoding = sensor_msgs::image_encodings::RGBA8;
               
               img_msg.step = img_msg.width * 4; // 1 Byte per 4 Channels of the RGBA format

               img_size = img_msg.step * img_msg.height;
               img_msg.data.resize(img_size);
                   NvMediaImageSurfaceMap surfaceMap;
               if (NvMediaImageLock(nvmedia_rgb_half_img_ptr->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
               {
                       unsigned char* buffer = (unsigned char*)surfaceMap.surface[0].mapping;
                       memcpy((char *)( &img_msg.data[0] ) , buffer , img_size);
                       gmsl_pub_img_.publish(ros_img_ptr_);
                       NvMediaImageUnlock(nvmedia_rgb_half_img_ptr->img);
               }
               //   cleanup
               CHECK_DW_ERROR(dwImage_destroy(&frame_yuv));       
               // return frame
               CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
               ros::spinOnce();
               loop_rate.sleep();
               ++count;
           }
       }
       catch (std::runtime_error &e)
       {
           std::cerr << e.what() << "\n";
       }
       
   }

};

//------------------------------------------------------------------------------
int main(int argc, const char *argv[])
{

   po::options_description desc{"Options"};
   desc.add_options()
       ("help,h", "Help screen")
       ("camera-type", po::value<std::string>()-> default_value("ar0231-rccb-bae-sf3324"), 
           "camera gmsl type (see sample_sensors_info for all available camera types on this platform)\n")
       ("camera-port", po::value<std::string>()-> default_value("a"), "Camera CSI port [default a]\n"
                             "a - port AB on px2, A on ddpx\n"
                             "c - port CD on px2, C on ddpx\n"
                             "e - port EF on px2, E on ddpx\n"
                             "g - G on ddpx only\n")
       ("tegra-slave", po::value<std::string>()-> default_value("0"),
           "Optional parameter used only for Tegra B, enables slave mode.\n")
       ("custom-board", po::value<std::string>()-> default_value("0"), "If true, then the configuration for board and camera "
                             "will be input from the config-file\n")
       ("custom-config", po::value<std::string>()-> default_value(""), "Set of custom board extra configuration\n");

   po::variables_map args;
   po::store(parse_command_line(argc, argv, desc), args);
   po::notify(args);

   CameraGMSL cam(args);
   cam.publish();    

   return 0;
}
