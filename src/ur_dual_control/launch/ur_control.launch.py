#!/usr/bin/env python3
"""
Copyright (c) 2025 Grupo Integrado de Ingenieria

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
  * Neither the name of Grupo Integrado de Ingenieria nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: Fabian Alvarez
2025, Grupo Integrado de Ingenieria
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # -------------------------------------------------------------------------
    # Launch configurations (arguments) for general settings
    # -------------------------------------------------------------------------
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # -------------------------------------------------------------------------
    # Robot-specific IP addresses (left and right robots)
    # -------------------------------------------------------------------------
    robot_ip_I = LaunchConfiguration("robot_ip_I")  # Left robot
    robot_ip_D = LaunchConfiguration("robot_ip_D")  # Right robot

    # -------------------------------------------------------------------------
    # Hardware and controller parameters
    # -------------------------------------------------------------------------
    kinematics_params_file_I = LaunchConfiguration("kinematics_params_file_I")
    kinematics_params_file_D = LaunchConfiguration("kinematics_params_file_D")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    initial_joint_controller_I = LaunchConfiguration("initial_joint_controller_I")
    initial_joint_controller_D = LaunchConfiguration("initial_joint_controller_D")
    activate_joint_controller_I = LaunchConfiguration("activate_joint_controller_I")
    activate_joint_controller_D = LaunchConfiguration("activate_joint_controller_D")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")

    # -------------------------------------------------------------------------
    # Tool communication parameters (for e-series robots)
    # -------------------------------------------------------------------------
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")

    # -------------------------------------------------------------------------
    # Define file paths and configurations using substitutions
    # -------------------------------------------------------------------------
    joint_limit_params = PathJoinSubstitution([
        FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"
    ])
    physical_params = PathJoinSubstitution([
        FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"
    ])
    visual_params = PathJoinSubstitution([
        FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"
    ])
    script_filename = PathJoinSubstitution([
        FindPackageShare("ur_client_library"), "resources", "external_control.urscript"
    ])
    input_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"
    ])
    output_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"
    ])

    # Generate the robot description (URDF) dynamically using xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_dual_control"), "urdf", "ur_dual_controlled.urdf.xacro",
        ]),
        " kinematics_parameters_file_I:=", kinematics_params_file_I,
        " kinematics_parameters_file_D:=", kinematics_params_file_D,
    ])
    robot_description = {"robot_description": robot_description_content}

    # Path to initial joint controllers configuration file
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur_dual_description"), "rviz", "urdf.rviz"
    ])

    # Define the update rate configuration file (depends on ur_type)
    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "config",
        ur_type.perform(context) + "_update_rate.yaml",
    ])

    # -------------------------------------------------------------------------
    # Nodes Definitions
    # -------------------------------------------------------------------------

    # Controller manager node for fake hardware
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    # UR control node for real hardware (not fake)
    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # Dashboard client nodes for both robots (only if enabled and not using fake hardware)
    dashboard_client_node_I = Node(
        package="ur_robot_driver",
        condition=IfCondition(AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip_I}],
        namespace="ur_dual_I",
    )

    dashboard_client_node_D = Node(
        package="ur_robot_driver",
        condition=IfCondition(AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip_D}],
        namespace="ur_dual_D",
    )

    # Tool communication node for the left robot (if enabled)
    tool_communication_node_I = Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[{"robot_ip": robot_ip_I, "tcp_port": tool_tcp_port, "device_name": tool_device_name}],
    )

    # URScript interface nodes for both robots
    urscript_interface_I = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip_I}],
        output="screen",
        namespace="ur_dual_I",
    )

    urscript_interface_D = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip_D}],
        output="screen",
        namespace="ur_dual_D",
    )

    # Controller stopper nodes for both robots
    controller_stopper_node_I = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller_I},
            {"consistent_controllers": [
                "io_and_status_controller_I",
                "force_torque_sensor_broadcaster_I",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster_I",
                "ur_configuration_controller",
            ]},
        ],
    )

    controller_stopper_node_D = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller_D},
            {"consistent_controllers": [
                "io_and_status_controller_D",
                "force_torque_sensor_broadcaster_D",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster_D",
                "ur_configuration_controller",
            ]},
        ],
    )

    # Robot state publisher node to broadcast TFs based on the robot description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz node for visualization (launched if 'launch_rviz' is true)
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # -------------------------------------------------------------------------
    # Spawn Controllers
    # -------------------------------------------------------------------------
    def controller_spawner(controllers, active=True):
        """Spawns the specified controllers with an option to mark them as inactive."""
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", controller_spawner_timeout,
            ] + inactive_flags + controllers,
        )

    # Define active and inactive controllers
    controllers_active = [
        "joint_state_broadcaster_I",
        "joint_state_broadcaster_D",
        "io_and_status_controller_I",
        "io_and_status_controller_D",
        "speed_scaling_state_broadcaster_I",
        "speed_scaling_state_broadcaster_D",
        "force_torque_sensor_broadcaster_I",
        "force_torque_sensor_broadcaster_D",
        "ur_configuration_controller_I",
        "ur_configuration_controller_D",
    ]
    controllers_inactive = ["forward_position_controller_I"]

    controller_spawners = [
        controller_spawner(controllers_active)
    ] + [controller_spawner(controllers_inactive, active=False)]

    # Initial joint controller spawners for left and right robots,
    # started or kept inactive based on 'activate_joint_controller'
    initial_joint_controller_spawner_started_I = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller_I, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        condition=IfCondition(activate_joint_controller_I),
    )

    initial_joint_controller_spawner_started_D = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller_D, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        condition=IfCondition(activate_joint_controller_D),
    )

    initial_joint_controller_spawner_stopped_I = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller_I, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller_I),
    )

    initial_joint_controller_spawner_stopped_D = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller_D, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller_D),
    )

    # -------------------------------------------------------------------------
    # Aggregate all nodes to start
    # -------------------------------------------------------------------------
    nodes_to_start = [
        control_node,
        ur_control_node,
        dashboard_client_node_I,
        dashboard_client_node_D,
        tool_communication_node_I,
        controller_stopper_node_I,
        controller_stopper_node_D,
        urscript_interface_I,
        urscript_interface_D,
        robot_state_publisher_node,
        initial_joint_controller_spawner_stopped_I,
        initial_joint_controller_spawner_stopped_D,
        initial_joint_controller_spawner_started_D,
        initial_joint_controller_spawner_started_I,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Declare Launch Arguments
    # -------------------------------------------------------------------------
    declared_arguments = []

    # UR robot type
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("robot_ip_I", description="IP address for the left robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enable safety limits controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="Margin for safety limits.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor for safety controller.",
        )
    )
    # General configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_dual_control",
            description='Package containing controller configuration (in the "config" folder).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_dual_control",
            description="Package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_dual_controlled.urdf.xacro",
            description="URDF/XACRO description file for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file_I",
            default_value=PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("description_package")),
                "config",
                LaunchConfiguration("ur_type"),
                "default_kinematics.yaml",
            ]),
            description="Kinematics configuration for the left robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF prefix for joint names (useful in multi-robot setups).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware (simulate robot states).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake sensor commands for simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initial robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller_I",
            default_value="scaled_joint_trajectory_controller_I",
            description="Initial controller for the left robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller_D",
            default_value="scaled_joint_trajectory_controller_D",
            description="Initial controller for the right robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller_I",
            default_value="true",
            description="Activate left robot's joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller_D",
            default_value="true",
            description="Activate right robot's joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Enable tool communication (only for e-series robots).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle characters for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle characters for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="Device file for tool communication (requires write permission).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="TCP port for bridging the tool's serial device.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",
            description="Tool voltage configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port for forwarding URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port for sending cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="Port for querying external_control URScript.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port for trajectory control.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
