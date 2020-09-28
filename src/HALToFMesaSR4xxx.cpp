#include <iostream>
#include <chrono>
#include <string>
#include <functional>

#include "definesSR.h"
#include "libMesaSR.h"

#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/console.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "hal_tof_mesa_sr4xxx/srv/get_device_info.hpp"
#include "hal_tof_mesa_sr4xxx/srv/get_amplitude_threshold.hpp"
#include "hal_tof_mesa_sr4xxx/srv/get_integration_time.hpp"
#include "hal_tof_mesa_sr4xxx/srv/get_serial.hpp"
#include "hal_tof_mesa_sr4xxx/srv/get_timeout.hpp"
#include "hal_tof_mesa_sr4xxx/srv/get_modulation_frequency.hpp"
#include "hal_tof_mesa_sr4xxx/srv/get_point_cloud.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_timeout.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_auto_exposure.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_amplitude_threshold.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_dual_integration.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_integration_time.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_non_ambiguity_mode.hpp"
#include "hal_tof_mesa_sr4xxx/srv/set_modulation_frequency.hpp"

using namespace std::chrono_literals;
namespace ph = std::placeholders;


#define SIZE_M 176*144

class HALToFMesaSR4xxxNode : public rclcpp::Node {
 public:

  HALToFMesaSR4xxxNode() : Node("PerceptionToF") {}

  bool init() {
      ip_address = declare_parameter("ip_address", "192.168.17.128");

      if (SR_OpenETH(&srCam, ip_address.c_str()) != 1) {
          RCLCPP_ERROR(get_logger(), "Failed to open device");
          return false;
      };
	  RCLCPP_INFO(get_logger(), "Successfully connected to device");
	  
    SR_SetModulationFrequency(srCam, MF_15MHz); 	
	SR_SetAmplitudeThreshold(srCam, 0x80);
	SR_SetIntegrationTime(srCam, 220);

      service_get_device_info = create_service<hal_tof_mesa_sr4xxx::srv::GetDeviceInfo>(
          "get_device_info",
          std::bind(&HALToFMesaSR4xxxNode::get_device_info, this, ph::_1, ph::_2));

      service_get_serial = create_service<hal_tof_mesa_sr4xxx::srv::GetSerial>(
          "get_serial",
          std::bind(&HALToFMesaSR4xxxNode::get_serial, this, ph::_1, ph::_2));

      service_get_amplitude_threshold = create_service<hal_tof_mesa_sr4xxx::srv::GetAmplitudeThreshold>(
          "get_amplitude_threshold",
          std::bind(&HALToFMesaSR4xxxNode::get_amplitude_threshold, this, ph::_1, ph::_2));

      service_set_amplitude_threshold = create_service<hal_tof_mesa_sr4xxx::srv::SetAmplitudeThreshold>(
          "set_amplitude_threshold",
          std::bind(&HALToFMesaSR4xxxNode::set_amplitude_threshold, this, ph::_1, ph::_2));

      service_get_integration_time = create_service<hal_tof_mesa_sr4xxx::srv::GetIntegrationTime>(
          "get_integration_time",
          std::bind(&HALToFMesaSR4xxxNode::get_integration_time, this, ph::_1, ph::_2));

      service_set_integration_time = create_service<hal_tof_mesa_sr4xxx::srv::SetIntegrationTime>(
          "set_integration_time",
          std::bind(&HALToFMesaSR4xxxNode::set_integration_time, this, ph::_1, ph::_2));

      service_set_timeout = create_service<hal_tof_mesa_sr4xxx::srv::SetTimeout>(
          "set_timeout",
          std::bind(&HALToFMesaSR4xxxNode::set_timeout, this, ph::_1, ph::_2));

      service_get_modulation_frequency = create_service<hal_tof_mesa_sr4xxx::srv::GetModulationFrequency>(
          "get_modulation_frequency",
          std::bind(&HALToFMesaSR4xxxNode::get_modulation_frequency, this, ph::_1, ph::_2));

      service_set_modulation_frequency = create_service<hal_tof_mesa_sr4xxx::srv::SetModulationFrequency>(
          "set_modulation_frequency",
          std::bind(&HALToFMesaSR4xxxNode::set_modulation_frequency, this, ph::_1, ph::_2));

      service_set_auto_exposure = create_service<hal_tof_mesa_sr4xxx::srv::SetAutoExposure>(
          "set_auto_exposure",
          std::bind(&HALToFMesaSR4xxxNode::set_auto_exposure, this, ph::_1, ph::_2));

      service_set_dual_integration = create_service<hal_tof_mesa_sr4xxx::srv::SetDualIntegration>(
          "set_dual_integration",
          std::bind(&HALToFMesaSR4xxxNode::set_dual_integration, this, ph::_1, ph::_2));

      service_set_non_ambiguity_mode = create_service<hal_tof_mesa_sr4xxx::srv::SetNonAmbiguityMode>(
          "set_non_ambiguity_mode",
          std::bind(&HALToFMesaSR4xxxNode::set_non_ambiguity_mode, this, ph::_1, ph::_2));

      service_get_point_cloud = create_service<hal_tof_mesa_sr4xxx::srv::GetPointCloud>(
          "get_point_cloud",
          std::bind(&HALToFMesaSR4xxxNode::get_point_cloud, this, ph::_1, ph::_2));

	  topic_point_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

      return true;
  }

  ~HALToFMesaSR4xxxNode() {
      if (srCam) SR_Close(srCam);
  }

 private:
  std::string ip_address;
  CMesaDevice *srCam = nullptr;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::GetDeviceInfo>::SharedPtr service_get_device_info;
  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::GetSerial>::SharedPtr service_get_serial;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::GetAmplitudeThreshold>::SharedPtr service_get_amplitude_threshold;
  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetAmplitudeThreshold>::SharedPtr service_set_amplitude_threshold;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::GetIntegrationTime>::SharedPtr service_get_integration_time;
  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetIntegrationTime>::SharedPtr service_set_integration_time;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetTimeout>::SharedPtr service_set_timeout;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::GetModulationFrequency>::SharedPtr service_get_modulation_frequency;
  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetModulationFrequency>::SharedPtr service_set_modulation_frequency;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetAutoExposure>::SharedPtr service_set_auto_exposure;
  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetDualIntegration>::SharedPtr service_set_dual_integration;
  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::SetNonAmbiguityMode>::SharedPtr service_set_non_ambiguity_mode;

  rclcpp::Service<hal_tof_mesa_sr4xxx::srv::GetPointCloud>::SharedPtr service_get_point_cloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr topic_point_cloud;

  int srSetMode(int currentMode, int flag, bool onOff)
  {
      int mode = currentMode & (~flag);
      if(onOff) mode |= flag;
      return mode;
  };

  void get_device_info(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetDeviceInfo::Request>,
                       std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetDeviceInfo::Response> response) {
      char buf[200]; //"VendorID:0x%04x, ProductID:0x%04x, Manufacturer:'%s', Product:'%s'"
      SR_GetDeviceString(srCam, buf, 200);

      std::string str(buf);
      std::stringstream ss(str);
      std::string token;

      std::getline(ss, token, ':');
      std::getline(ss, response->vendor_id, ',');
      std::getline(ss, token, ':');
      std::getline(ss, response->product_id, ',');
      std::getline(ss, token, ':');
      std::getline(ss, response->manufacturer, ',');
      std::getline(ss, token, ':');
      std::getline(ss, response->product);
  }

  void get_serial(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetSerial::Request>,
                  std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetSerial::Response> response) {
      response->serial = SR_ReadSerial(srCam);
  }

  void get_amplitude_threshold(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetAmplitudeThreshold::Request>,
                               std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetAmplitudeThreshold::Response> response) {
      response->amplitude_threshold = SR_GetAmplitudeThreshold(srCam);
  }

  void set_amplitude_threshold(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetAmplitudeThreshold::Request> request,
                               std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetAmplitudeThreshold::Response> response) {
      SR_SetAmplitudeThreshold(srCam, request->amplitude_threshold);
      response->success = true;
  }

  void get_integration_time(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetIntegrationTime::Request>,
                            std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetIntegrationTime::Response> response) {
      unsigned char int_time = SR_GetIntegrationTime(srCam);
      response->integration_us = 300 + int_time * 100; //Formula valid for SR4xxx cameras
  }

  void set_integration_time(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetIntegrationTime::Request> request,
                            std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetIntegrationTime::Response> response) {
      if (((request->integration_us - 300) / 100) > 255) {
          response->success = false;
      } else {
          unsigned char int_time = std::ceil((request->integration_us - 300) / 100);
          SR_SetIntegrationTime(srCam, int_time);
          response->success = true;
      }
  }

  void set_timeout(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetTimeout::Request> request,
                   std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetTimeout::Response> response) {
      SR_SetTimeout(srCam, request->timeout_ms);
      response->success = true;
  }

  void get_modulation_frequency(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetModulationFrequency::Request>,
                                std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetModulationFrequency::Response> response) {
      int mod_freq_code = SR_GetModulationFrequency(srCam);
      switch (mod_freq_code) {
          case MF_15MHz:
              response->modulation_frequency_mhz = 15;
              break;
          case MF_14_5MHz:
              response->modulation_frequency_mhz = 14.5;
              break;
          case MF_15_5MHz:
              response->modulation_frequency_mhz = 15.5;
              break;
          case MF_29MHz:
              response->modulation_frequency_mhz = 29;
              break;
          case MF_30MHz:
              response->modulation_frequency_mhz = 30;
              break;
          case MF_31MHz:
              response->modulation_frequency_mhz = 31;
              break;
          default:
              response->modulation_frequency_mhz = 0;
      }
  }

  void set_modulation_frequency(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetModulationFrequency::Request> request,
                                std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetModulationFrequency::Response> response) {
      int doublefreq = std::round(request->modulation_frequency_mhz * 2);

      switch (doublefreq) {
          case 30: //15*2
              SR_SetModulationFrequency(srCam, MF_15MHz);
              response->success = true;
              break;
          case 29: //14.5*2
              SR_SetModulationFrequency(srCam, MF_14_5MHz);
              response->success = true;
              break;
          case 31: //15.5*2
              SR_SetModulationFrequency(srCam, MF_15_5MHz);
              response->success = true;
              break;
          case 60: //30*2
              SR_SetModulationFrequency(srCam, MF_30MHz);
              response->success = true;
              break;
          case 58: //29*2
              SR_SetModulationFrequency(srCam, MF_29MHz);
              response->success = true;
              break;
          case 62: //31*2
              SR_SetModulationFrequency(srCam, MF_31MHz);
              response->success = true;
              break;
          default:
              response->success = false;
      }
  }

  void set_auto_exposure(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetAutoExposure::Request> request,
                         std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetAutoExposure::Response> response) {
      SR_SetAutoExposure(srCam,
                         request->min_int_time,
                         request->max_int_time,
                         request->percent_over_pos,
                         request->desired_pos);
      response->success = true;
  }

  void set_dual_integration(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetDualIntegration::Request> request,
                            std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetDualIntegration::Response> response) {
    if (request->ratio >100){
        response->success=false;
    } else {
        SR_SetDualIntegrationTime(srCam, request->ratio);
        response->success = true;
    }
  }

  void set_non_ambiguity_mode(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetNonAmbiguityMode::Request> request,
                              std::shared_ptr<hal_tof_mesa_sr4xxx::srv::SetNonAmbiguityMode::Response> response) {
      SR_SetMode(srCam, srSetMode(SR_GetMode(srCam), AM_NO_AMB, request->enable));
      response->success=true;
  }


  void get_point_cloud(const std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetPointCloud::Request>,
                       std::shared_ptr<hal_tof_mesa_sr4xxx::srv::GetPointCloud::Response> response) {

      float xTOF[SIZE_M], yTOF[SIZE_M], zTOF[SIZE_M];
      int retVal = SR_Acquire(srCam);
      if(retVal<0) return;
      SR_CoordTrfFlt(srCam, xTOF, yTOF, zTOF, sizeof(float), sizeof(float), sizeof(float));

	  pcl::PointCloud<pcl::PointXYZ> cloud;
	  
	  cloud.width    = SIZE_M;
	  cloud.height   = 1;
	  cloud.is_dense = false;
	  cloud.points.resize (cloud.width * cloud.height);

	  for (long i = 0; i<SIZE_M; i++){
		cloud.points[i].x = xTOF[i];
		cloud.points[i].y = yTOF[i];
		cloud.points[i].z = zTOF[i];
		//std::cout << xTOF[i] << " \t" << yTOF[i] << " \t" << zTOF[i] << " \t";
	  }

	  pcl::io::savePCDFileASCII ("cloud.pcd", cloud);
	  pcl::toROSMsg(cloud, response->point_cloud);
	  response->point_cloud.header.frame_id = "map";
	  response->point_cloud.header.stamp = now();
	  topic_point_cloud->publish(response->point_cloud);

    // http://wiki.ros.org/pcl/Overview
    // https://pcl.readthedocs.io/projects/tutorials/en/latest/writing_pcd.html#writing-pcd

  }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<HALToFMesaSR4xxxNode>();
    if (not node->init()) rclcpp::shutdown();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
