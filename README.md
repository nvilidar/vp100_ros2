# VP100/T10 ROS2 DRIVER

## How to install ROS2
[install ros2](https://index.ros.org/doc/ros2/Installation)

## How to Create a ROS workspace

[Create a workspace](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace)

you also can with this:

    1)  $mkdir -p ~/lidar_ros2_ws/src
        $cd ~/lidar_ros2_ws/src
    2)  $cd..
    3)  $colcon build --symlink-install
    4)  $source ./install/setup.bash

## How to build Lidar ROS2 Package
### 1.Get the ros2 code
    1) Clone this project to your catkin's workspace src folder
    	(1). git clone https://github.com/nvilidar/vp100_ros2.git  
             or
             git clone https://gitee.com/nvilidar/vp100_ros2.git
    	(2). git chectout master

    2) download the code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin    
### 2.Copy the ros code
    1) Copy the ros source file to the "~/lidar_ros2_ws/src"
    2) Running "colcon build --symlink-install" to build lidar_node and lidar_client
### 3.Serialport configuration
if you use the lidar device name,you must give the permissions to user.
```shell
whoami
```
get the user name.like ubuntu.
```shell
sudo usermod -a -G dialout ubuntu
```
ubuntu is the user name.
```shell
sudo reboot   
```

## ROS2 Parameter Configuration
### 1. Lidar Support
    the baudrate can be 115200bsp or 230400bps 

## How to Run LIDAR ROS2 Package
### 1. Run LIDAR node and view in the rviz
------------------------------------------------------------
```shell
ros2 launch lidar_ros2 lidar_launch_view.py 
```

### 2. Run node and view using test application
------------------------------------------------------------
```shell
ros2 launch lidar_ros2 lidar_launch.py 
ros2 run lidar_ros2 lidar_ros2_client
```

#### 3. echo scan topic
------------------------------------------------------------
```shell
ros2 topic echo /scan
```

## LIDAR ROS Parameter
|  value   |  information  |
|  :----:    | :----:  |
| serial_baud  | if use serialport,the lidar's serialport |
| serial_name  | if use serialport,the lidar's port name |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| counterclockwise  | lidar's point counterclockwise|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:15.0 meters|
| range_min  | lidar's min measure distance,default:0.001 meters|
| angle_corp_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°-60° and 90°-120° points in the view|