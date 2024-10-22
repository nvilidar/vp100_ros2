/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 19:34:30
 * @Description  : 
 */
#ifndef __ROS2_CONVERT_H__
#define __ROS2_CONVERT_H__

#include "lidar.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nvistar{
class ROS2Convert{
public:
  #pragma pack(push)
  #pragma pack(1)
    typedef struct{
      std::string frame_id;          //frame id 
      bool        counterclockwise;  //counter clockwise 
      double      angle_min;         //angle min 
      double      angle_max;         //angle max 
      double      range_min;         //range min
      double      range_max;         //range max 
      bool        resolution_fixed;  //fixed resolution
      std::string angle_corp_string; //angle ignore
    }lidar_ros_config_t;
  #pragma pack(pop)

  ROS2Convert();
  ~ROS2Convert();

  void lidar_raw_to_ros2(lidar_scan_period_t lidar_raw, lidar_ros_config_t config, sensor_msgs::msg::LaserScan &ros_scan);
private:
  double angle_to_ros(bool counterclockwise_flag,double angle);         //angle to ros 
  std::vector<float> angle_corp_get(std::string angle_string);          //angle corp get 
  bool angle_corp_flag(float angle,std::vector<float> angle_corp_list); //angle corp 
};
}

#endif