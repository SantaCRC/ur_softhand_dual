#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Variables para ur_dual_I
    ur_dual_I_ur_type = LaunchConfiguration("ur_dual_I_ur_type")
    ur_dual_I_robot_ip = LaunchConfiguration("ur_dual_I_robot_ip")
    ur_dual_I_use_mock_hardware = LaunchConfiguration("ur_dual_I_use_mock_hardware")
    ur_dual_I_mock_sensor_commands = LaunchConfiguration("ur_dual_I_mock_sensor_commands")
    ur_dual_I_kinematics_parameters_file = LaunchConfiguration("ur_dual_I_kinematics_parameters_file")

    # Variables para ur_dual_D
    ur_dual_D_ur_type = LaunchConfiguration("ur_dual_D_ur_type")
    ur_dual_D_robot_ip = LaunchConfiguration("ur_dual_D_robot_ip")
    ur_dual_D_use_mock_hardware = LaunchConfiguration("ur_dual_D_use_mock_hardware")
    ur_dual_D_mock_sensor_commands = LaunchConfiguration("ur_dual_D_mock_sensor_commands")
    ur_dual_D_kinematics_parameters_file = LaunchConfiguration("ur_dual_D_kinematics_parameters_file")

    headless_mode = LaunchConfiguration("headless_mode")

    # Se genera la descripción del robot a partir del archivo xacro, pasando los parámetros para cada brazo
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_dual_control"),
                    "urdf",
                    "ur_dual_controlled.urdf.xacro",
                ]
            ),
            " ",
            "ur_dual_I_robot_ip:=", ur_dual_I_robot_ip,
            " ",
            "ur_dual_D_robot_ip:=", ur_dual_D_robot_ip,
            " ",
            "ur_dual_I_ur_type:=", ur_dual_I_ur_type,
            " ",
            "ur_dual_D_ur_type:=", ur_dual_D_ur_type,
            " ",
            "ur_dual_I_use_mock_hardware:=", ur_dual_I_use_mock_hardware,
            " ",
            "ur_dual_D_use_mock_hardware:=", ur_dual_D_use_mock_hardware,
            " ",
            "ur_dual_I_kinematics_parameters_file:=", ur_dual_I_kinematics_parameters_file,
            " ",
            "ur_dual_D_kinematics_parameters_file:=", ur_dual_D_kinematics_parameters_file,
            " ",
            "ur_dual_I_mock_sensor_commands:=", ur_dual_I_mock_sensor_commands,
            " ",
            "ur_dual_D_mock_sensor_commands:=", ur_dual_D_mock_sensor_commands,
            " ",
            "headless_mode:=", headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_I_ur_type",
            description="Type/series of the UR robot for ur_dual_I.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_D_ur_type",
            description="Type/series of the UR robot for ur_dual_D.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_I_robot_ip",
            default_value="192.168.0.101",
            description="IP address for ur_dual_I.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_D_robot_ip",
            default_value="192.168.0.100",
            description="IP address for ur_dual_D.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_I_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "alice_calibration.yaml",
                ]
            ),
            description="The calibration configuration for ur_dual_I.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_D_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "bob_calibration.yaml",
                ]
            ),
            description="The calibration configuration for ur_dual_D.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_I_use_mock_hardware",
            default_value="false",
            description="Start ur_dual_I with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_D_use_mock_hardware",
            default_value="false",
            description="Start ur_dual_D with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_I_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for ur_dual_I's sensors used for simple simulations.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_dual_D_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for ur_dual_D's sensors used for simple simulations.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()