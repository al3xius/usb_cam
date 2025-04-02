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

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, color_format_name_, custom_auto_exposure_parameter_ ;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, focus_;
  bool autofocus_, custom_auto_exposure_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;
  
  // Service to update camera parameters from dictionary
  ros::ServiceServer service_update_params_;

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
      node_("~")
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


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the color format
    UsbCam::color_format color_format = UsbCam::color_format_from_string(color_format_name_);
    if (color_format == UsbCam::COLOR_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown color format '%s'", color_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, color_format, image_width_,
         image_height_, framerate_);

    // Load and apply V4L parameters from dictionary
    loadV4LParametersFromDict();




    // check auto focus
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
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }

};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
