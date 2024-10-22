import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  share_dir = get_package_share_directory('lidar_ros2')
  rviz_config_file = os.path.join(share_dir, 'rviz','lidar.rviz')

  rviz2_node = Node(
                      package='rviz2',
                      executable='rviz2',
                      name='rviz2',
                      arguments=['-d', rviz_config_file],
                    )
  
  lidar_launch = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource([
      get_package_share_directory('lidar_ros2'),
      '/launch/lidar_launch.py'
    ])
  )

  launch_description = LaunchDescription()

  launch_description.add_action(lidar_launch)
  launch_description.add_action(rviz2_node)

  return launch_description