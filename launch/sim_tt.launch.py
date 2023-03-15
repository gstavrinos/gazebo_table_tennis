#!/usr/bin/env python3
import os
import math
import random 
from launch import LaunchDescription 
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_pal.substitutions import LoadFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    table_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/table_description.launch.py"]
        ),
    )

    racket_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/racket_description.launch.py"]
        ),
    )

    ball_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/ball_description.launch.py"]
        ),
    )

    spawn_table = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_table",
            arguments=["-entity", "table", "-topic", "table_description", "-x", "2.64", "-y", "0", "-z", "0"],
            output="screen",
            )

    table_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="table_odom_to_tf",
            parameters=[
                {"odom_topic": "/p3d/table_odom"},
                ]
            )

    spawn_racket = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_racket",
            arguments=["-entity", "racket", "-topic", "racket_description", "-x", "1.34", "-y", "0", "-z", "0.8"],
            output="screen",
            )

    racket_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="racket_odom_to_tf",
            parameters=[
                {"odom_topic": "/p3d/racket_odom"},
                ]
            )

    spawn_ball = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_ball",
            arguments=["-entity", "ball", "-topic", "ball_description", "-x", "1.4", "-y", "0.3", "-z", "1.06"],
            output="screen",
            )

    ball_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="ball_odom_to_tf",
            parameters=[
                {"odom_topic": "/p3d/ball_odom"},
                ]
            )

    play_motion = ExecuteProcess(
        cmd=["ros2", "launch", "tiago_bringup", "play_motion.launch.py"],
        output="screen"
    )

    return LaunchDescription(
        [

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
                ),
        ),

        GroupAction(
            actions=[
                table_description_launch,
                spawn_table,
                table_tf,
            ]
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_table,
                on_exit=[
                    TimerAction(
                        period = 20.0,
                        actions = [
                            GroupAction(
                                actions=[racket_description_launch, spawn_racket, racket_tf]
                            ),
                        ]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_racket,
                on_exit=[
                    GroupAction(
                        actions=[ball_description_launch, spawn_ball, ball_tf]
                        ),
                ],
            )
        ),

        ],
    )

