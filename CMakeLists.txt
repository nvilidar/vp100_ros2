cmake_minimum_required(VERSION 3.5)
project(lidar_ros2)

# set base info
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find package 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# add include 
include_directories(
  ${catkin_INCLUDE_DIRS}
  sdk/include
  ros2/include
)

# add src
FILE(GLOB LIDAR_SDK_SRC 
  "sdk/src/lidar/lidar_protocol.cpp"
  "sdk/src/lidar.cpp"
  "sdk/src/interface/console/interface_console.cpp"
)
if(WIN32)  
  list(APPEND LIDAR_SDK_SRC "sdk/src/interface/serial/win/interface_serial.cpp")  
elseif(UNIX)  
  list(APPEND LIDAR_SDK_SRC "sdk/src/interface/serial/unix/interface_serial.cpp")  
endif()

#link client
add_executable(lidar_ros2_client ros2/src/ros2_client.cpp)
ament_target_dependencies(lidar_ros2_client
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
)

#link node 
add_executable(lidar_ros2_node ros2/src/ros2_node.cpp  ros2/src/ros2_convert.cpp ${LIDAR_SDK_SRC})
ament_target_dependencies(lidar_ros2_node
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
)

#install
install(TARGETS lidar_ros2_node lidar_ros2_client
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params rules rviz
        DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
