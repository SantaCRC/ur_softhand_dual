import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable


def generate_launch_description():
    # --- Declaración de argumentos ---
    ur_type = LaunchConfiguration("ur_type")

    # Parámetros para brazo 0
    arm_0_robot_ip = LaunchConfiguration("arm_0_robot_ip")
    arm_0_script_command_port = LaunchConfiguration("arm_0_script_command_port")
    arm_0_trajectory_port = LaunchConfiguration("arm_0_trajectory_port")
    arm_0_reverse_port = LaunchConfiguration("arm_0_reverse_port")
    arm_0_script_sender_port = LaunchConfiguration("arm_0_script_sender_port")
    arm_0_prefix = LaunchConfiguration("arm_0_prefix")

    # Parámetros para brazo 1
    arm_1_robot_ip = LaunchConfiguration("arm_1_robot_ip")
    arm_1_script_command_port = LaunchConfiguration("arm_1_script_command_port")
    arm_1_trajectory_port = LaunchConfiguration("arm_1_trajectory_port")
    arm_1_reverse_port = LaunchConfiguration("arm_1_reverse_port")
    arm_1_script_sender_port = LaunchConfiguration("arm_1_script_sender_port")
    arm_1_prefix = LaunchConfiguration("arm_1_prefix")

    declared_arguments = [
        # Generales
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        # Brazo 0
        DeclareLaunchArgument("arm_0_robot_ip", default_value="10.113.36.100"),
        DeclareLaunchArgument("arm_0_script_command_port", default_value="50004"),
        DeclareLaunchArgument("arm_0_trajectory_port", default_value="50003"),
        DeclareLaunchArgument("arm_0_reverse_port", default_value="50001"),
        DeclareLaunchArgument("arm_0_script_sender_port", default_value="50002"),
        DeclareLaunchArgument("arm_0_prefix", default_value="arm_0_"),
        # Brazo 1
        DeclareLaunchArgument("arm_1_robot_ip", default_value="10.113.36.247"),
        DeclareLaunchArgument("arm_1_script_command_port", default_value="50014"),
        DeclareLaunchArgument("arm_1_trajectory_port", default_value="50013"),
        DeclareLaunchArgument("arm_1_reverse_port", default_value="50011"),
        DeclareLaunchArgument("arm_1_script_sender_port", default_value="50012"),
        DeclareLaunchArgument("arm_1_prefix", default_value="arm_1_"),
    ]

    # --- Definición de rutas ---
    ur_launch_dir = get_package_share_directory("ur_robot_driver")
    dual_control_dir = get_package_share_directory("ur_dual_control")
    dual_description_dir = get_package_share_directory("ur_dual_description")

    # --- Descripción del robot (URDF) ---
    # Se genera una descripción combinada pasando los prefijos de ambos brazos al archivo XACRO.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([dual_description_dir, "urdf", "ur_dual.urdf.xacro"]),
        " ",
        "ur_type:=", ur_type,
        " ",
        "arm_0_prefix:=", arm_0_prefix,
        " ",
        "arm_1_prefix:=", arm_1_prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

    # Nodo que publica el robot_state (URDF) para ambos brazos
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # --- Controlador único que combina ambos brazos ---
    # Se incluye el launch file "ur_control.launch.py" pasando como argumentos los parámetros
    # tanto del brazo 0 como del brazo 1. (Se asume que dicho launch file está preparado para
    # gestionar un controlador que combine ambos brazos.)
    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "launch", "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": arm_0_robot_ip,  # Se utiliza la IP del brazo 0 (o la que corresponda)
            "kinematics_params_file": PathJoinSubstitution([
                dual_control_dir, "config", "ur_dual_kinematics.yaml"
            ]),
            "headless_mode": "true",
            "non_blocking_read": "true",
            "keep_alive_count": "10",
            "tf_prefix": "ur_dual_D_",

            # Parámetros para el brazo 0
            "arm_0_script_command_port": arm_0_script_command_port,
            "arm_0_trajectory_port": arm_0_trajectory_port,
            "arm_0_reverse_port": arm_0_reverse_port,
            "arm_0_script_sender_port": arm_0_script_sender_port,
            "arm_0_prefix": arm_0_prefix,

            # Parámetros para el brazo 1
            "arm_1_script_command_port": arm_1_script_command_port,
            "arm_1_trajectory_port": arm_1_trajectory_port,
            "arm_1_reverse_port": arm_1_reverse_port,
            "arm_1_script_sender_port": arm_1_script_sender_port,
            "arm_1_prefix": arm_1_prefix,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        controller_manager,
    ])
