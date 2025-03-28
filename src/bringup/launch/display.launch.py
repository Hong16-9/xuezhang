import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='bringup').find('bringup')
    default_model_path = os.path.join(pkg_share, 'src/description/xiaoche.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')
    world_path=os.path.join(pkg_share, 'world/RMUC.sdf')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bringup',
            '-topic', 'robot_description',
            '-x','2.0',
            '-y','0.0',
            '-z','0.1'
        ],
        output='screen'
    )

    tf2_publish_cmd = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )

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
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        tf2_publish_cmd,
        map_to_baselink_node,
        odom_to_baselink_node,
        laser_to_pointcloud_node,
        map_to_topic_node,
        local_costmap_node
    ])
