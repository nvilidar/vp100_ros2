import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  share_dir = get_package_share_directory('lidar_ros2')
  parameter_file = LaunchConfiguration('params_file')

  params_declare = DeclareLaunchArgument(
                                          'params_file',
                                          default_value=os.path.join(share_dir, 'params', 'lidar.yaml'),
                                          description='FPath to the ROS2 parameters file to use.'
                                        )

  driver_node = LifecycleNode(
                                package='lidar_ros2',
                                executable='lidar_ros2_node',
                                name='lidar_ros2_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                              )
  tf2_node = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                  )

  launch_description = LaunchDescription()

  launch_description.add_action(params_declare)
  launch_description.add_action(driver_node)
  launch_description.add_action(tf2_node)

  return launch_description