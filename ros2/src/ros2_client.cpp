/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 18:33:18
 * @Description  : ros2 client 
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    int count = scan->scan_time / scan->time_increment;
    printf("[info]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[info]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        printf("[info]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_ros2_client");

    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                                "scan", rclcpp::SensorDataQoS(), scanCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}