/*
 * @Version      : V1.0
 * @Date         : 2024-10-18 15:24:38
 * @Description  : ros2 node publish 
 */
#include "interface/serial/interface_serial.hpp"
#include "interface/console/interface_console.hpp"
#include "ros2_convert.hpp"
#include "lidar.hpp"
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <inttypes.h>
#include <thread>
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

//define 
nvistar::InterfaceSerial *_serial;
nvistar::Lidar  *_lidar;
nvistar::ROS2Convert *_convert;

//read param define
#define READ_PARAM(TYPE, NAME, VAR, VALUE) VAR = VALUE; \
       	node->declare_parameter<TYPE>(NAME, VAR); \
       	node->get_parameter(NAME, VAR);

//comm interface 
int serial_write(const uint8_t* data,int length){
  return _serial->serial_write(data, length);
}
int serial_read(uint8_t* data,int length){
  return _serial->serial_read(data, length);
}
void serial_flush(){
  _serial->serial_flush();
}
//timestamp 
uint64_t get_stamp(){
  uint64_t nanoseconds = 0;
  nanoseconds = rclcpp::Clock().now().nanoseconds();
	return nanoseconds;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc,argv); 

  auto node = rclcpp::Node::make_shared("lidar_ros2_node");

  nvistar::ROS2Convert::lidar_ros_config_t config;
  std::string serial_name;
  int serial_baud;
  int timeout_count = 0;
  //sync para form launch 
  READ_PARAM(std::string, "serial_name", (serial_name), "/dev/ttyUSB0");
  READ_PARAM(int, "serial_baud", (serial_baud), 230400);
  READ_PARAM(std::string, "frame_id", (config.frame_id), "laser_frame"); 
  READ_PARAM(bool, "resolution_fixed", (config.resolution_fixed), false); 
  READ_PARAM(bool, "counterclockwise", (config.counterclockwise), true); 
  READ_PARAM(double, "angle_max", (config.angle_max), 180); 
  READ_PARAM(double, "angle_min", (config.angle_min), -180.0); 
  READ_PARAM(double, "range_max", (config.range_max), 15.0); 
  READ_PARAM(double, "range_min", (config.range_min), 0.001); 
  READ_PARAM(std::string, "angle_corp_string", (config.angle_corp_string), ""); 
  //start function 
  _serial = new nvistar::InterfaceSerial();
  _lidar = new nvistar::Lidar();
  _convert = new nvistar::ROS2Convert();
  //callback function
  nvistar::lidar_interface_t  _interface = {
    {
      serial_write,
      serial_read,
      serial_flush,
    },
    get_stamp
  };
  //serial open 
  bool ret = _serial->serial_open(serial_name, serial_baud);
  //lidar register 
  if(ret){
    _lidar->lidar_register(&_interface);
    RCLCPP_INFO(node->get_logger(), "lidar is scanning...\n");
  }else{
    RCLCPP_ERROR(node->get_logger(), "lidar serial open failed!");
    _serial->serial_close();
  }
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  rclcpp::WallRate rate(50);
  //loop to get point
  while (ret && (rclcpp::ok())) {
    nvistar::lidar_scan_period_t lidar_raw_scan;
    sensor_msgs::msg::LaserScan lidar_ros_scan;

    nvistar::lidar_scan_status_t status = _lidar->lidar_get_scandata(lidar_raw_scan);
    switch(status){
      case nvistar::LIDAR_SCAN_OK:{
        timeout_count = 0;
        _convert->lidar_raw_to_ros2(lidar_raw_scan, config, lidar_ros_scan);
        try {
          scan_pub->publish(lidar_ros_scan);
        }catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "lidar ros2 throw error:%s !",e.what());
        }
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_MOTOR_LOCK: {
        timeout_count = 0;
        RCLCPP_ERROR(node->get_logger(), "lidar motor lock!");
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT: {
        timeout_count = 0;
        RCLCPP_ERROR(node->get_logger(), "lidar motor short circuit!");
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_UP_NO_POINT: {
        timeout_count = 0;
        RCLCPP_ERROR(node->get_logger(), "lidar upboard no points!");
        break;
      }
      case nvistar::LIDAR_SCAN_TIMEOUT: {
        RCLCPP_ERROR(node->get_logger(), "lidar data timeout!");
        //reconnect 
        timeout_count++;
        if(timeout_count >= 10){
          timeout_count = 0;
          _serial->serial_reopen();
          RCLCPP_INFO(node->get_logger(), "lidar serial reopen!\n");
        }
        break;
      }
      default:{
        break;
      }
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }
  RCLCPP_INFO(node->get_logger(), "lidar is stoping...\n");
  //ros stop 
  delete _lidar;
  _lidar = nullptr;
  delete _serial;
  _serial = nullptr;
  delete _convert;
  _convert = nullptr;
}

