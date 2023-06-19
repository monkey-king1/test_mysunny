// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "camera_tis/camera_tis.hpp"


// #include <exception>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>


namespace camera_tis
{

using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::SetParametersResult;

/**
 * @brief Gstreamer pipeline.
 *
 * Raw image format, resolution and fps are defined here.
 * Then with videoscale plugin, it is scaled down to a lower resolution to reduce overall CPU usage.
 * The pipeline use appsink plugin to pass image date asynchronously.
 */


CameraTis::CameraTis(const rclcpp::NodeOptions & options)
: Node("camera_tis_node", options)
{
  /**
   * @brief Const expression for image size infomation.
   *
   */
  WIDTH = 2048;
  HEIGHT = 1544;
  SIZE = WIDTH * HEIGHT;
  FPS = 30;
  //VIEW_WIDTH = camdata.camer_size_view_width;
  //VIEW_HEIGHT = camdata.camer_size_view_height;

//_param_imagepos = std::make_shared<rclcpp::AsyncParametersClient>(this, "laser_imagepos_node");
  _pub = this->create_publisher<Image>(_pub_name, rclcpp::SensorDataQoS());
  _initialize_camera();

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

CameraTis::~CameraTis()
{
  try {
  #ifdef SHOW_OUTPUT_FPS
    _threadmodbus.join();
  #endif
    _handle.reset();
    
    _thread.join();
   
    _pub.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void CameraTis::_declare_parameters()
{
//ParameterDescriptor pd;
//pd.read_only = true;
  this->declare_parameter("exposure_time", 1000);
  this->declare_parameter("width", WIDTH);
  this->declare_parameter("height", HEIGHT);
  this->declare_parameter("fps", FPS);
  this->declare_parameter("view_width", VIEW_WIDTH);
  this->declare_parameter("view_height", VIEW_HEIGHT);
  this->declare_parameter("power", false, ParameterDescriptor(), true);
}

void CameraTis::_initialize_camera()
{
#ifdef SHOW_OUTPUT_FPS
  b_modbusconnect=false;

  _threadmodbus = std::thread(&CameraTis::_modbus, this, 1502);
#endif

  _declare_parameters();


  


  //初始化相机


  _thread = std::thread(&CameraTis::_spin, this);

  // ROS parameter callback handle.
 
}

#ifdef SHOW_OUTPUT_FPS
void CameraTis::_modbus(int port)
{
  while (rclcpp::ok()) 
  {
    ctx = modbus_new_tcp(NULL, 1502);
    if (!ctx) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create modbus camfps.");
      rclcpp::shutdown();
      return;
    }
    if (modbus_connect(ctx) == -1)
    {
      modbus_free(ctx);
      usleep(5);
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "connect modbus camfps succeed.");
    b_modbusconnect=true;
    break;
  }
}
#endif

void CameraTis::_spin()
{

  int frame = 0;
  std::string v4l2_cmd = "media-ctl -d /dev/media0 --set-v4l2 '\"m00_b_mvcam 5-003b\":0[fmt:Y8_1X8/" +
                            std::to_string(2048) + "x" + std::to_string(1544) +
                            "@1/" + std::to_string(45) + " field:none]'";

    system(v4l2_cmd.c_str());

    std::string v4l2src_pipeline = "v4l2src io-mode=dmabuf device=/dev/video0 ! video/x-raw, format=(string)GRAY8, width=(int)" +
                                    std::to_string(2048) + ", height=(int)" + std::to_string(1544) +
                                    " ! appsink";

    cv::VideoCapture cap(v4l2src_pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) 
    {
      std::cerr << "Failed to open camera" << std::endl;
    }

  while (rclcpp::ok()) {

    cv::Mat img;
      cap.read(img);
      if(img.empty())
      continue;
      //std::cout<<"aaa"<<std::endl;
      std::cout<<img.size()<<std::endl;
      auto ptr = std::make_unique<Image>();
      ptr->header.stamp = this->now();
      ptr->header.frame_id = std::to_string(frame++);
      ptr->height = HEIGHT;
      ptr->width = WIDTH;
      ptr->encoding = "mono8";
      ptr->is_bigendian = false;
      ptr->step = WIDTH;
      ptr->data.resize(SIZE);
      memcpy(ptr->data.data(), img.data, SIZE);
     
    #ifdef SHOW_OUTPUT_FPS
      if(b_modbusconnect==true)
      {
          static bool b_timest=0;
          static auto timest_fps=ptr->header.stamp;
          auto timeed_fps=ptr->header.stamp;
          if(b_timest==0)
          {
            b_timest=1;
            timest_fps=ptr->header.stamp;
          }
          else
          {
            timeed_fps=ptr->header.stamp;
            double timest=(double)timest_fps.sec+(double)timest_fps.nanosec/1000000000;
            double timeed=(double)timeed_fps.sec+(double)timeed_fps.nanosec/1000000000;
            double time=timeed-timest;
            int fps=(int)((double)1.0/time*100.0);
            u_int16_t tab_reg[1];
            tab_reg[0]=(u_int16_t)fps;
            int rc=modbus_write_registers(ctx,0x0c,1,tab_reg);
            if(rc!=1)
            {
              RCLCPP_INFO(this->get_logger(), "Failed to send modbus camfps.");
            }
            timest_fps=timeed_fps;
          }
      }
    #endif
      _pub->publish(std::move(ptr));
    
  }

}


}  // namespace camera_tis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_tis::CameraTis)
