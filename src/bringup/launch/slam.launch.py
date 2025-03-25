import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    
    map_to_baselink_node = launch_ros.actions.Node(
        package='map_to_baselink',
        executable='map_to_baselink',       
    )

    odom_to_baselink_node = launch_ros.actions.Node(
        package='odom_to_baselink',
        executable='odom_to_baselink'
    )

    laser_to_pointcloud_node = launch_ros.actions.Node(
        package='laser_to_pointcloud',
        executable='laser_to_pointcloud'
    )

    map_to_topic_node = launch_ros.actions.Node(
        package='map_to_topic',
        executable='map_to_topic'
    )

    local_costmap_node = launch_ros.actions.Node(
        package='local_costmap',
        executable='local_costmap'
    )

    return launch.LaunchDescription([

        map_to_baselink_node,
        odom_to_baselink_node,
        laser_to_pointcloud_node,
        map_to_topic_node,
        local_costmap_node

    ])
