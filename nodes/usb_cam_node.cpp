/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <vo_autoexpose/vo_autoexpose.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <usb_cam_auto_exposure/CameraSettingsConfig.h>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;
  ExpGainController controller; 

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<usb_cam_auto_exposure::CameraSettingsConfig> server_;
  dynamic_reconfigure::Server<usb_cam_auto_exposure::CameraSettingsConfig>::CallbackType f_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, color_format_name_, custom_auto_exposure_parameter_ ;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, focus_, current_custom_exposure_, current_gain_;
  bool autofocus_, custom_auto_exposure_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  // Camera reconnection parameters
  int reconnect_delay_ms_;
  int max_reconnect_delay_ms_;
  bool camera_connected_;
  
  // Connection status publisher
  ros::Publisher camera_status_pub_;
  std_msgs::String camera_status_msg_;
  
  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;
  
  // Service to update camera parameters from dictionary
  ros::ServiceServer service_update_params_;

  // Auto exposure parameters
  int min_exposure_, max_exposure_, target_exposure_;
  double target_brightness_, brightness_tolerance_;
  int auto_exposure_frames_skip_;
  int frames_since_last_adjustment_;

  // Method to apply V4L parameters from a dictionary
  void loadV4LParametersFromDict() {
    XmlRpc::XmlRpcValue v4l_params;
    if (node_.getParam("v4l_params", v4l_params) && v4l_params.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_INFO("Loading V4L parameters from dictionary");
      
      for (XmlRpc::XmlRpcValue::iterator it = v4l_params.begin(); it != v4l_params.end(); ++it) {
        std::string param_name = it->first;
        XmlRpc::XmlRpcValue& value = it->second;
        
        // Handle different types of parameters
        switch (value.getType()) {
          case XmlRpc::XmlRpcValue::TypeInt:
            ROS_INFO("Setting V4L parameter %s = %d", param_name.c_str(), static_cast<int>(value));
            cam_.set_v4l_parameter(param_name, static_cast<int>(value));
            break;
            
          case XmlRpc::XmlRpcValue::TypeBoolean:
            ROS_INFO("Setting V4L parameter %s = %d", param_name.c_str(), static_cast<bool>(value) ? 1 : 0);
            cam_.set_v4l_parameter(param_name, static_cast<bool>(value) ? 1 : 0);
            break;
            
          case XmlRpc::XmlRpcValue::TypeString:
            {
              std::string str_value = static_cast<std::string>(value);
              // Try to convert string to integer if it looks like a number
              char* end;
              long int_value = strtol(str_value.c_str(), &end, 10);
              if (*end == '\0') {
                // Conversion successful, it's a numeric string
                ROS_INFO("Setting V4L parameter %s = %ld", param_name.c_str(), int_value);
                cam_.set_v4l_parameter(param_name, static_cast<int>(int_value));
              } else {
                ROS_INFO("Setting V4L parameter %s = %s", param_name.c_str(), str_value.c_str());
                cam_.set_v4l_parameter(param_name, str_value);
              }
            }
            break;
            
          default:
            ROS_WARN("Unsupported parameter type for %s", param_name.c_str());
            break;
        }
        
        // Special handling for auto controls
        if (param_name == "autofocus") {
          bool new_autofocus = false;
          if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            new_autofocus = static_cast<bool>(value);
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            new_autofocus = static_cast<int>(value) != 0;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            std::string str_value = static_cast<std::string>(value);
            new_autofocus = (str_value == "true" || str_value == "1");
          }
          
          if (new_autofocus != autofocus_) {
            autofocus_ = new_autofocus;
            if (autofocus_) {
              cam_.set_auto_focus(1);
              cam_.set_v4l_parameter("focus_auto", 1);
            } else {
              cam_.set_v4l_parameter("focus_auto", 0);
            }
          }
        }

        if (param_name == custom_auto_exposure_parameter_) {
          current_custom_exposure_ = static_cast<int>(value);
        }

        if (param_name == "gain") {
          current_gain_ = static_cast<int>(value);
        }
      }
    }
  }
  
  // Service callback to update parameters
  bool updateParametersCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    loadV4LParametersFromDict();
    return true;
  }

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }

  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~"), 
      camera_connected_(false),
      reconnect_delay_ms_(500),  // Start with a 500ms reconnect delay
      max_reconnect_delay_ms_(30000),  // Maximum 30 second delay between reconnect attempts
      min_exposure_(0),
      max_exposure_(255),
      target_exposure_(100),
      target_brightness_(128.0),
      brightness_tolerance_(10.0),
      auto_exposure_frames_skip_(5),
      frames_since_last_adjustment_(0)
  {
    ROS_INFO("Starting usb_cam node with auto exposure...");
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // possible values: yuv420p, yuv422p
    node_.param("color_format", color_format_name_, std::string("yuv422p"));

    // parameters for custom auto exposure
    node_.param("custom_auto_exposure", custom_auto_exposure_, false);
    node_.param("custom_auto_exposure_parameter", custom_auto_exposure_parameter_, std::string("exposure_time_absolute"));

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);
    service_update_params_ = node_.advertiseService("update_parameters", &UsbCamNode::updateParametersCallback, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }

    // Setup exposure controller with default values
    // We'll update with real camera parameters after connection
    // setupExposureController();

    // Create a publisher for camera connection status
    camera_status_pub_ = node_.advertise<std_msgs::String>("camera_status", 1, true);
    
    // get reconnection parameters
    node_.param("reconnect_delay_ms", reconnect_delay_ms_, 500);
    node_.param("max_reconnect_delay_ms", max_reconnect_delay_ms_, 30000);
    
    // Get auto exposure parameters
    node_.param("target_brightness", target_brightness_, 128.0);
    node_.param("brightness_tolerance", brightness_tolerance_, 10.0);
    node_.param("min_exposure", min_exposure_, 0);
    node_.param("max_exposure", max_exposure_, 255);
    node_.param("auto_exposure_frames_skip", auto_exposure_frames_skip_, 5);

    // Set initial connection status
    publish_camera_status("CONNECTING");

    // Try to start the camera
    initialize_camera();

    // Set up dynamic reconfigure
    f_ = boost::bind(&UsbCamNode::dynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);
  }

  // New method to set up the exposure controller with camera parameters
  void setupExposureController() {
    // Get calibration values from parameters or use defaults
    Vector5f calib;
    std::vector<double> calib_values;
    if (node_.getParam("calibration_values", calib_values) && calib_values.size() == 5) {
      calib << calib_values[0], calib_values[1], calib_values[2], calib_values[3], calib_values[4];
    } else {
      ROS_WARN("Using default calibration values. For best results, provide 'calibration_values' parameter.");
      calib << 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    // Get focal length values
    double F, focal_length_x, focal_length_y;
    node_.param("focal_length", F, 2.0);
    
    // Check if we have calibrated camera info
    if (cinfo_->isCalibrated()) {
      sensor_msgs::CameraInfo info = cinfo_->getCameraInfo();
      focal_length_x = info.K[0]; // fx is in the calibration matrix K[0]
      focal_length_y = info.K[4]; // fy is in the calibration matrix K[4]
      
      // If no values in calibration matrix, try to get from parameters
      if (focal_length_x == 0.0) {
        node_.param("focal_length_x", focal_length_x, 865.909537);
      }
      if (focal_length_y == 0.0) {
        node_.param("focal_length_y", focal_length_y, 865.909537);
      }
    } else {
      // No calibration, use parameters or defaults
      node_.param("focal_length_x", focal_length_x, 865.909537);
      node_.param("focal_length_y", focal_length_y, 865.909537);
    }

    // Configure gamma values
    VectorNcorrect gamma_values;
    std::vector<double> gamma_params;
    if (node_.getParam("gamma_values", gamma_params) && gamma_params.size() == gamma_values.size()) {
      for (size_t i = 0; i < gamma_params.size(); i++) {
        gamma_values[i] = gamma_params[i];
      }
    } else {
      // gamma_values << 1.0/1.9, 1.0/1.5, 1.0/1.2, 1.0, 1.2, 1.5, 1.9;
    }

    gamma_values << 0, 1, 2, 3, 4, 5, 6;

    // Get camera identification
    std::string camera_id;
    node_.param("camera_id", camera_id, std::string("left"));
    
    ROS_INFO("Configuring exposure controller: %s with dimensions %dx%d, F: %.1f, fx: %.1f, fy: %.1f", 
             camera_id.c_str(), image_width_, image_height_, F, focal_length_x, focal_length_y);

    // Add camera to controller
    controller.add_camera(camera_id, image_height_, image_width_, FLT_DIG, focal_length_x, focal_length_y, calib);
    controller.set_gamma_values(gamma_values);
    controller.init();
  }

  // Method to publish camera status
  void publish_camera_status(const std::string& status) {
    camera_status_msg_.data = status;
    camera_status_pub_.publish(camera_status_msg_);
    
    // Log the status change as well
    ROS_INFO("Camera status: %s", status.c_str());
  }
  
  // New method to initialize the camera and handle errors gracefully
  bool initialize_camera() {
    try {
      ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
          image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

      // set the IO method
      UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
      if(io_method == UsbCam::IO_METHOD_UNKNOWN)
      {
        ROS_ERROR("Unknown IO method '%s'", io_method_name_.c_str());
        publish_camera_status("ERROR");
        return false;
      }

      // set the pixel format
      UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
      if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
      {
        ROS_ERROR("Unknown pixel format '%s'", pixel_format_name_.c_str());
        publish_camera_status("ERROR");
        return false;
      }

      // set the color format
      UsbCam::color_format color_format = UsbCam::color_format_from_string(color_format_name_);
      if (color_format == UsbCam::COLOR_FORMAT_UNKNOWN)
      {
        ROS_ERROR("Unknown color format '%s'", color_format_name_.c_str());
        publish_camera_status("ERROR");
        return false;
      }

      // start the camera - wrapped in try/catch to handle failures
      try {
        cam_.start(video_device_name_.c_str(), io_method, pixel_format, color_format, image_width_,
             image_height_, framerate_);
             
        // Load and apply V4L parameters from dictionary
        loadV4LParametersFromDict();

        // Set autofocus if specified
        if (autofocus_)
        {
          cam_.set_auto_focus(1);
          cam_.set_v4l_parameter("focus_auto", 1);
        }
        else
        {
          cam_.set_v4l_parameter("focus_auto", 0);
          if (focus_ >= 0)
          {
            cam_.set_v4l_parameter("focus_absolute", focus_);
          }
        }
        
        // If we got here, camera is working
        camera_connected_ = true;
        publish_camera_status("CONNECTED");
        
        // Reset reconnect delay when successfully connected
        reconnect_delay_ms_ = 500;
        
        // If we successfully connected, update the controller with current parameters
        // if (camera_connected_) {
        //   setupExposureController();
        // }
        
        return true;
      } 
      catch (std::exception &e) {
        ROS_ERROR("Exception starting camera: %s", e.what());
        publish_camera_status("DISCONNECTED");
        return false;
      }
    }
    catch (std::exception &e) {
      ROS_ERROR("Exception during camera initialization: %s", e.what());
      publish_camera_status("ERROR");
      return false;
    }
  }
  
  // Try to reconnect to the camera with exponential backoff
  void attempt_reconnection() {
    if (camera_connected_) return; // Already connected
    
    ROS_INFO("Attempting to reconnect to camera after %d ms", reconnect_delay_ms_);
    
    if (initialize_camera()) {
      ROS_INFO("Successfully reconnected to camera");
      camera_connected_ = true;
      publish_camera_status("CONNECTED");
    } else {
      ROS_ERROR("Failed to reconnect to camera");
      // Implement exponential backoff with a ceiling
      reconnect_delay_ms_ = std::min(reconnect_delay_ms_ * 2, max_reconnect_delay_ms_);
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  // Calculate the average brightness of an image
  double calculateAverageBrightness(const sensor_msgs::Image& img) {
    // Only process a subset of pixels for efficiency
    const int sample_step = 10; // Sample every 10th pixel in both dimensions
    long int sum = 0;
    long int count = 0;

    // Handle different encoding formats
    if (img.encoding == "rgb8" || img.encoding == "bgr8") {
      for (size_t y = 0; y < img.height; y += sample_step) {
        for (size_t x = 0; x < img.width; x += sample_step) {
          size_t index = y * img.step + x * 3;
          // Average of RGB channels
          int r = img.data[index];
          int g = img.data[index + 1];
          int b = img.data[index + 2];
          // Convert RGB to luminance (standard coefficients)
          int luminance = 0.299 * r + 0.587 * g + 0.114 * b;
          sum += luminance;
          count++;
        }
      }
    } else if (img.encoding == "mono8" || img.encoding == "8UC1") {
      // Grayscale image
      for (size_t y = 0; y < img.height; y += sample_step) {
        for (size_t x = 0; x < img.width; x += sample_step) {
          size_t index = y * img.step + x;
          sum += img.data[index];
          count++;
        }
      }
    } else if (img.encoding == "yuyv" || img.encoding == "uyvy") {
      // YUV format - just sample Y component
      for (size_t y = 0; y < img.height; y += sample_step) {
        for (size_t x = 0; x < img.width; x += sample_step) {
          size_t index = y * img.step + x * 2; // YUV formats have 2 bytes per pixel
          sum += img.data[index]; // Y component
          count++;
        }
      }
    } else {
      // If we can't determine the format, just sample and average all bytes
      for (size_t y = 0; y < img.height; y += sample_step) {
        for (size_t x = 0; x < img.width * (img.step / img.width); x += sample_step) {
          size_t index = y * img.step + x;
          if (index < img.data.size()) {
            sum += img.data[index];
            count++;
          }
        }
      }
    }

    return count > 0 ? static_cast<double>(sum) / count : 0.0;
  }

  // Adjust exposure based on image brightness
  void adjustExposure(double current_brightness) {
    // Skip if auto exposure is disabled
    if (!custom_auto_exposure_) return;

    ROS_INFO("Adjusting exposure based on brightness: %.1f, current: %d", current_brightness, current_custom_exposure_);

    // Calculate brightness error
    double brightness_error = target_brightness_ - current_brightness;

    ROS_INFO("Brightness Error %.1f.", 
      brightness_error);
    
    // Skip adjustment if within tolerance
    if (std::abs(brightness_error) <= brightness_tolerance_) {
      return;
    }
    
    // Calculate proportional adjustment
    double gain = 1.5; // Adjust this value to control responsiveness
    node_.param("exposure_gain", gain, 1.5); // Allow param override
    
    int adjustment = static_cast<int>(brightness_error * gain);
    
    // Apply adjustment based on camera type
    int new_exposure;
    if (custom_auto_exposure_parameter_ == "exposure_absolute" || 
        custom_auto_exposure_parameter_ == "exposure") {
      // Standard behavior - higher value means more exposure
      new_exposure = current_custom_exposure_ - adjustment;
    } else {
      // Some cameras may use inverted values
      new_exposure = current_custom_exposure_ + adjustment;
    }

    // Clamp exposure to valid range
    new_exposure = std::max(min_exposure_, std::min(max_exposure_, new_exposure));
    
    // Only update if value changed
    if (new_exposure != current_custom_exposure_) {
      ROS_INFO("Adjusting %s: %d -> %d (brightness: %.1f, target: %.1f, error: %.1f)",
              custom_auto_exposure_parameter_.c_str(), current_custom_exposure_, new_exposure, 
              current_brightness, target_brightness_, brightness_error);
      try {
        cam_.set_v4l_parameter(custom_auto_exposure_parameter_, new_exposure);
        current_custom_exposure_ = new_exposure;
      } catch (std::exception &e) {
        ROS_ERROR("Failed to set exposure: %s", e.what());
      }
    }
  }

  bool take_and_send_image()
  {
    // grab the image
    try {
      bool success = cam_.grab_image(&img_);
      if (!success) {
        return false;
      }
    }
    catch (std::exception &e) {
      ROS_ERROR("Exception during image capture: %s", e.what());
      return false;
    }

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);

    // Handle auto exposure if enabled
    // if (custom_auto_exposure_) {
    //   // frames_since_last_adjustment_++;
    //   // if (frames_since_last_adjustment_ >= auto_exposure_frames_skip_) {
    //   //   frames_since_last_adjustment_ = 0;
    //   //   double avg_brightness = calculateAverageBrightness(img_);
    //   //   adjustExposure(avg_brightness);
    //   // }

    //   cv_bridge::CvImagePtr cv_ptr;
    //   cv_ptr = cv_bridge::toCvCopy(img_,"mono8");

    //   controller.add_frame(0,cv_ptr->image,float(current_custom_exposure_)/1000000,controller.factor_2_dB(current_gain_),cv_ptr->header.seq,cv_ptr->header.stamp.toSec());


    //   // Get the new exposure and gain values
    //   ExposureParameters new_exposure_params = controller.update_params(GRADINFO_SCORE,EV_CORRECTIONS,OPTIFLOW_ESTIM);

    //   int new_gain = static_cast<int>(controller.dB_2_factor(new_exposure_params.gain));
    //   // convert gain from db to range 0 to 255
    //   new_gain = std::max(0, std::min(20, new_gain));

    //   int new_exposure = static_cast<int>(1000000*new_exposure_params.exposure);
    //   new_exposure = std::max(min_exposure_, std::min(max_exposure_, new_exposure));

    //   // update parameters
    //   if (new_exposure != current_custom_exposure_) {
    //     ROS_INFO("Updating exposure to %d", new_exposure);
    //     cam_.set_v4l_parameter(custom_auto_exposure_parameter_, new_exposure);
    //     current_custom_exposure_ = new_exposure;
    //   }

    //   if (new_gain != current_gain_) {
    //     ROS_INFO("Updating gain to %d", new_gain);
    //     cam_.set_v4l_parameter("gain", new_gain);
    //     current_gain_ = new_gain;
    //   }
    // }

    // Update dynamic reconfigure with current values
    // This ensures the GUI always shows the actual camera settings
    // usb_cam::CameraSettingConfig config;
    // config.exposure = current_custom_exposure_;
    // config.gain = current_gain_;
    // config.auto_exposure = custom_auto_exposure_;
    // config.target_brightness = target_brightness_;
    // config.brightness_tolerance = brightness_tolerance_;
    // server_.updateConfig(config);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    ros::Time last_reconnect_attempt = ros::Time::now();
    
    while (node_.ok())
    {
      // Check if camera is connected
      if (camera_connected_) {
        if (cam_.is_capturing()) {
          if (!take_and_send_image()) {
            ROS_WARN("Failed to capture image. Camera may be disconnected.");
            
            // Check if camera is still there
            if (!cam_.check_camera_connected()) {
              ROS_ERROR("Camera disconnected.");
              camera_connected_ = false;
              publish_camera_status("DISCONNECTED");
              cam_.disconnect(); // Clean disconnection
            }
          }
        }
      } else {
        // If not connected, attempt to reconnect after delay
        last_reconnect_attempt = ros::Time::now();          
        attempt_reconnection();

        // if ((ros::Time::now() - last_reconnect_attempt) > reconnect_duration) {        
        //   ros::Duration reconnect_duration(reconnect_delay_ms_ / 1000.0);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }

    return true;
  }

  // Dynamic reconfigure callback
  void dynamicReconfigureCallback(usb_cam_auto_exposure::CameraSettingsConfig &config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure called with level %d", level);
    
    try {
      cam_.set_v4l_parameter(custom_auto_exposure_parameter_, config.exposure);
      current_custom_exposure_ = config.exposure;
    } catch (std::exception &e) {
      ROS_ERROR("Failed to set exposure: %s", e.what());
    }    
  }
};   
}     


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam_auto_exposure");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
