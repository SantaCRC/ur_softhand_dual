import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)

def generate_launch_description():
    # --- Declaración de argumentos generales ---
    ur_type = LaunchConfiguration("ur_type")
    
    # Brazo I
    robot_ip_I = LaunchConfiguration("robot_ip_I")
    script_command_port_I = LaunchConfiguration("script_command_port_I")
    trajectory_port_I = LaunchConfiguration("trajectory_port_I")
    reverse_port_I = LaunchConfiguration("reverse_port_I")
    script_sender_port_I = LaunchConfiguration("script_sender_port_I")
    prefix_I = LaunchConfiguration("prefix_I")
    
    # Brazo D
    robot_ip_D = LaunchConfiguration("robot_ip_D")
    script_command_port_D = LaunchConfiguration("script_command_port_D")
    trajectory_port_D = LaunchConfiguration("trajectory_port_D")
    reverse_port_D = LaunchConfiguration("reverse_port_D")
    script_sender_port_D = LaunchConfiguration("script_sender_port_D")
    prefix_D = LaunchConfiguration("prefix_D")
    
    declared_arguments = [
        # Generales
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        # Parámetros para el brazo I
        DeclareLaunchArgument("robot_ip_I", default_value="10.113.36.100"),
        DeclareLaunchArgument("script_command_port_I", default_value="50004"),
        DeclareLaunchArgument("trajectory_port_I", default_value="50003"),
        DeclareLaunchArgument("reverse_port_I", default_value="50001"),
        DeclareLaunchArgument("script_sender_port_I", default_value="50002"),
        DeclareLaunchArgument("prefix_I", default_value="ur_dual_I_"),
        # Parámetros para el brazo D
        DeclareLaunchArgument("robot_ip_D", default_value="10.113.36.247"),
        DeclareLaunchArgument("script_command_port_D", default_value="50014"),
        DeclareLaunchArgument("trajectory_port_D", default_value="50013"),
        DeclareLaunchArgument("reverse_port_D", default_value="50011"),
        DeclareLaunchArgument("script_sender_port_D", default_value="50012"),
        DeclareLaunchArgument("prefix_D", default_value="ur_dual_D_"),
    ]
    
    # --- Definición de rutas a paquetes ---
    dual_control_dir = get_package_share_directory("ur_dual_control")
    dual_description_dir = get_package_share_directory("ur_dual_description")
    
    # --- Descripción del robot (URDF) ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([dual_description_dir, "urdf", "ur_dual.urdf.xacro"]),
        " ",
        "ur_type:=", ur_type,
        " ",
        "prefix_I:=", prefix_I,
        " ",
        "prefix_D:=", prefix_D,
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # Nodo que publica el estado del robot (URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # --- Controlador para brazo I ---
    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_control_dir, "launch", "ur_control.launch.py")
        ),
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
            "initial_joint_controller": "scaled_joint_trajectory_controller_I",
            "script_command_port": script_command_port_I,
            "trajectory_port": trajectory_port_I,
            "reverse_port": reverse_port_I,
            "script_sender_port": script_sender_port_I,
        }.items(),
    )
    
    return LaunchDescription(
        declared_arguments + [
            #robot_state_publisher,
            controller_manager,
        ]
    )

if __name__ == "__main__":
    generate_launch_description()
