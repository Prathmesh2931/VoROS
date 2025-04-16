#!/usr/bin/env python3

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare 
from launch_ros.actions import Node


def generate_launch_description():
    turtle=FindPackageShare('testbot_sim')
    share_dir=FindPackageShare('custom_service')

    yaw =Node(package='custom_service',executable='yaw.py')

    move=Node(package='custom_service',executable='movebot_text.py')

    hear=Node(package='custom_service',executable='speech_to_command.py')

    bot=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtle, 'launch/turtlebot3_robocon.launch.py'
            ])]))
    
    return  LaunchDescription([
        bot,
        yaw,
        # hear,
        # move

    ])