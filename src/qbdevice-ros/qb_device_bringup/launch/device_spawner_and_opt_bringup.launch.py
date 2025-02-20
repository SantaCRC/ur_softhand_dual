# Software License Agreement: BSD 3-Clause License

# Copyright (c) 2018-2024, qbrobotics®
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
#   products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gui",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_id",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_type",
            default_value='qbmove',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ignore_timestamp",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "default_controller",
        )
    )

    # Initialize Arguments
    prefix = LaunchConfiguration("prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    device_name = LaunchConfiguration("device_name")
    device_id = LaunchConfiguration("device_id")
    device_type = LaunchConfiguration("device_type")
    robot_package = LaunchConfiguration("robot_package")
    ignore_timestamp = LaunchConfiguration("ignore_timestamp")

    #Variable names
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_name",
            default_value=[device_type,".urdf.xacro"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_name",
            default_value=[device_name,"_joint_state_broadcaster"],
        )
    )
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare([robot_package,"_description"]),
                    "urdf",
                    LaunchConfiguration('urdf_name'),
                ]
            ),
            " ",
            "robot_name:=",
            device_name,
            " ",
            "namespace:=",
            device_name,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare([robot_package,"_description"]), "rviz","display.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,ignore_timestamp],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    gui_node = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        name="rqt_joint_trajectory_controller",
        namespace=device_name,
        condition=IfCondition(use_gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration('controller_name')],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=device_name,
        arguments=[LaunchConfiguration('default_controller')],

    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_gui_node_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gui_node],
        )
    )

    nodes = [
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_gui_node_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)