#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="gazebo_table_tennis",
            description="The package that includes the table description.",
        )
    )

    easy_collision = LaunchConfiguration("easy_collision")

    easy_collision_arg = DeclareLaunchArgument(
        "easy_collision",
        default_value="false"
    )


    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="table.urdf.xacro",
            description="The table description urdf.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    robot_description_content = Command(
         [
             PathJoinSubstitution([FindExecutable(name="xacro")]),
             " ",
             PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
             " ",
             "easy_collision:=",
             PythonExpression(["'", easy_collision,"'"])
             ]
    )

    return LaunchDescription(
        declared_arguments + 
        [
        easy_collision_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="table_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": robot_description_content, "publish_frequency": 20.0, "ignore_timestamp": False,
            "frame_prefix":"",}],
            remappings=[("/robot_description","/table_description"
            )]),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="table_joint_state_publisher",
            remappings=[("/robot_description","/table_description")],
            parameters=[{"use_sim_time": True,}],
            output="screen")
        ]
    )
