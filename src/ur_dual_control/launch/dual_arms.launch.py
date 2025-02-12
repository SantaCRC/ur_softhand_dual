import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    # --- Argumentos de IP ---
    arm_0_ip_arg = DeclareLaunchArgument(
        "arm_0_robot_ip", default_value="10.113.36.100",
        description="IP del brazo 0"
    )
    arm_1_ip_arg = DeclareLaunchArgument(
        "arm_1_robot_ip", default_value="10.113.36.247",
        description="IP del brazo 1"
    )

    # --- Argumentos de puertos para cada brazo ---
    arm_0_script_cmd_arg = DeclareLaunchArgument("arm_0_script_command_port", default_value="50004")
    arm_0_traj_arg = DeclareLaunchArgument("arm_0_trajectory_port", default_value="50003")
    arm_0_reverse_arg = DeclareLaunchArgument("arm_0_reverse_port", default_value="50001")
    arm_0_script_sender_arg = DeclareLaunchArgument("arm_0_script_sender_port", default_value="50002")

    arm_1_script_cmd_arg = DeclareLaunchArgument("arm_1_script_command_port", default_value="50014")
    arm_1_traj_arg = DeclareLaunchArgument("arm_1_trajectory_port", default_value="50013")
    arm_1_reverse_arg = DeclareLaunchArgument("arm_1_reverse_port", default_value="50011")
    arm_1_script_sender_arg = DeclareLaunchArgument("arm_1_script_sender_port", default_value="50012")

    # --- Rutas ---
    ur_launch_dir = get_package_share_directory("ur_robot_driver")
    ur_dual_control_dir = get_package_share_directory("ur_dual_control")

    # --- Grupo Brazo 0 ---
    # 1) Iniciamos ur_control.launch.py en el namespace /arm_0
    # 2) Spawneamos controladores en /arm_0/controller_manager
    arm_0 = GroupAction([
        PushRosNamespace("arm_0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur_launch_dir, "launch", "ur_control.launch.py")
            ),
            launch_arguments={
                "robot_ip":             LaunchConfiguration("arm_0_robot_ip"),
                "ur_type":              "ur5e",
                "kinematics_params_file": PathJoinSubstitution([
                                          ur_dual_control_dir,
                                          "config",
                                          "ur_dual_D_kinematics.yaml"
                                        ]),
                "headless_mode":        "true",
                "non_blocking_read":    "true",
                "keep_alive_count":     "10",
                "script_command_port":  LaunchConfiguration("arm_0_script_command_port"),
                "trajectory_port":      LaunchConfiguration("arm_0_trajectory_port"),
                "reverse_port":         LaunchConfiguration("arm_0_reverse_port"),
                "script_sender_port":   LaunchConfiguration("arm_0_script_sender_port"),
            }.items(),
        ),
        # --- Spawners en /arm_0 ---
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_scaled_joint_trajectory_controller",
            namespace="arm_0",
            arguments=[
                "scaled_joint_trajectory_controller",
                "-c", "/arm_0/controller_manager",
                "--controller-manager-timeout", "10"
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_joint_state_broadcaster",
            namespace="arm_0",
            arguments=[
                "joint_state_broadcaster", 
                "io_and_status_controller",
                "speed_scaling_state_broadcaster",
                "force_torque_sensor_broadcaster",
                # Comentarlo si no lo usas:
                "ur_configuration_controller",
                "-c", "/arm_0/controller_manager",
                "--controller-manager-timeout", "10"
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_forward_position_controller",
            namespace="arm_0",
            arguments=[
                "forward_position_controller",
                "--inactive",
                "-c", "/arm_0/controller_manager",
                "--controller-manager-timeout", "10"
            ],
        ),
    ])

    # --- Grupo Brazo 1 ---
    arm_1 = GroupAction([
        PushRosNamespace("arm_1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur_launch_dir, "launch", "ur_control.launch.py")
            ),
            launch_arguments={
                "robot_ip":             LaunchConfiguration("arm_1_robot_ip"),
                "ur_type":              "ur5e",
                "controllers_file":     PathJoinSubstitution([
                                          ur_dual_control_dir,
                                          "config",
                                          "ur_dual_I_controllers.yaml"
                                        ]),
                "kinematics_params_file": PathJoinSubstitution([
                                          ur_dual_control_dir,
                                          "config",
                                          "ur_dual_I_kinematics.yaml"
                                        ]),
                "headless_mode":        "true",
                "non_blocking_read":    "true",
                "keep_alive_count":     "10",
                "script_command_port":  LaunchConfiguration("arm_1_script_command_port"),
                "trajectory_port":      LaunchConfiguration("arm_1_trajectory_port"),
                "reverse_port":         LaunchConfiguration("arm_1_reverse_port"),
                "script_sender_port":   LaunchConfiguration("arm_1_script_sender_port"),
            }.items(),
        ),
        # --- Spawners en /arm_1 ---
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_scaled_joint_trajectory_controller",
            namespace="arm_1",
            arguments=[
                "scaled_joint_trajectory_controller",
                "-c", "/arm_1/controller_manager",
                "--controller-manager-timeout", "10"
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_joint_state_broadcaster",
            namespace="arm_1",
            arguments=[
                "joint_state_broadcaster", 
                "io_and_status_controller",
                "speed_scaling_state_broadcaster",
                "force_torque_sensor_broadcaster",
                # Comentarlo si no lo usas:
                "ur_configuration_controller",
                "-c", "/arm_1/controller_manager",
                "--controller-manager-timeout", "10"
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_forward_position_controller",
            namespace="arm_1",
            arguments=[
                "forward_position_controller",
                "--inactive",
                "-c", "/arm_1/controller_manager",
                "--controller-manager-timeout", "10"
            ],
        ),
    ])

    return LaunchDescription([
        arm_0_ip_arg,
        arm_1_ip_arg,
        arm_0_script_cmd_arg,
        arm_0_traj_arg,
        arm_0_reverse_arg,
        arm_0_script_sender_arg,
        arm_1_script_cmd_arg,
        arm_1_traj_arg,
        arm_1_reverse_arg,
        arm_1_script_sender_arg,
        arm_0,
        arm_1
    ])