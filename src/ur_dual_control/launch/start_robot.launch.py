"""
Launch file for setting up dual UR5e robots with independent controllers.

Author: Fabian Alvarez (alvarez.fabian@outlook.com)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """Generate launch description for dual UR5e robot setup."""
    # General parameters
    ur_type = LaunchConfiguration("ur_type")
    
    # Parameters for robot I (left)
    robot_ip_I = LaunchConfiguration("robot_ip_I")
    script_command_port_I = LaunchConfiguration("script_command_port_I")
    trajectory_port_I = LaunchConfiguration("trajectory_port_I")
    reverse_port_I = LaunchConfiguration("reverse_port_I")
    script_sender_port_I = LaunchConfiguration("script_sender_port_I")
    prefix_I = LaunchConfiguration("prefix_I")
    
    # Parameters for robot D (right)
    robot_ip_D = LaunchConfiguration("robot_ip_D")
    script_command_port_D = LaunchConfiguration("script_command_port_D")
    trajectory_port_D = LaunchConfiguration("trajectory_port_D")
    reverse_port_D = LaunchConfiguration("reverse_port_D")
    script_sender_port_D = LaunchConfiguration("script_sender_port_D")
    prefix_D = LaunchConfiguration("prefix_D")
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="Type of UR robot."),
        DeclareLaunchArgument("robot_ip_I", default_value="10.113.36.100", description="IP address of left robot."),
        DeclareLaunchArgument("script_command_port_I", default_value="50004", description="Script command port for left robot."),
        DeclareLaunchArgument("trajectory_port_I", default_value="50003", description="Trajectory control port for left robot."),
        DeclareLaunchArgument("reverse_port_I", default_value="50001", description="Reverse communication port for left robot."),
        DeclareLaunchArgument("script_sender_port_I", default_value="50002", description="Script sender port for left robot."),
        DeclareLaunchArgument("prefix_I", default_value="ur_dual_I_", description="Prefix for left robot."),
        DeclareLaunchArgument("robot_ip_D", default_value="10.113.36.247", description="IP address of right robot."),
        DeclareLaunchArgument("script_command_port_D", default_value="50014", description="Script command port for right robot."),
        DeclareLaunchArgument("trajectory_port_D", default_value="50013", description="Trajectory control port for right robot."),
        DeclareLaunchArgument("reverse_port_D", default_value="50011", description="Reverse communication port for right robot."),
        DeclareLaunchArgument("script_sender_port_D", default_value="50012", description="Script sender port for right robot."),
        DeclareLaunchArgument("prefix_D", default_value="ur_dual_D_", description="Prefix for right robot."),
    ]
    
    # Retrieve package path
    dual_control_dir = get_package_share_directory("ur_dual_control")
    
    # Include controller launch file for both robots
    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dual_control_dir, "launch", "ur_control.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip_I": robot_ip_I,
            "robot_ip_D": robot_ip_D,
            "kinematics_params_file_I": PathJoinSubstitution([
                dual_control_dir, "config", "ur_dual_I_kinematics.yaml"
            ]),
            "kinematics_params_file_D": PathJoinSubstitution([
                dual_control_dir, "config", "ur_dual_D_kinematics.yaml"
            ]),
            "controllers_file": PathJoinSubstitution([
                dual_control_dir, "config", "ur_dual_controllers.yaml"
            ]),
            "headless_mode": "true",
            "non_blocking_read": "true",
            "keep_alive_count": "10",
            "tf_prefix": "ur_dual_D_",
            "script_command_port": script_command_port_I,
            "trajectory_port": trajectory_port_I,
            "reverse_port": reverse_port_I,
            "script_sender_port": script_sender_port_I,
        }.items(),
    )
    
    return LaunchDescription(declared_arguments + [controller_manager])

if __name__ == "__main__":
    generate_launch_description()
