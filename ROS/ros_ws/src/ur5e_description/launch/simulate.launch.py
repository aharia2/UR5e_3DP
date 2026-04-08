import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('ur5e_description')
    ur_description_share = get_package_share_directory('ur_description')
    
    default_model_path = os.path.join(pkg_share, 'urdf', 'ur5e.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'ur5e.rviz')
  
    robot_description_content = Command(['xacro ', LaunchConfiguration('model'), ' ur_type:=ur5e', ' name:=ur'])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        )
    ])
