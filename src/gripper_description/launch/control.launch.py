import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    # Directorio de la descripción del robot y configuración
    robot_description_dir = get_package_share_directory('ur_dual_description')
    onrobot_description_dir = get_package_share_directory('onrobot_gripper')
    controller_yaml = os.path.join(onrobot_description_dir, 'config', 'controller.yaml')

    # Ruta al archivo XACRO
    urdf_xacro = os.path.join(robot_description_dir, 'urdf', 'ur_dual.urdf.xacro')
    # Procesar el archivo XACRO para obtener el URDF en formato XML
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', urdf_xacro])

    # Nodo para el controller_spawner con los parámetros cargados
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        namespace='onrobot_2fg7',
        output='screen',
        respawn=False,
        arguments=['lef_finger_joint_position_controller', 'joint_state_controller'],
        parameters=[controller_yaml]
    )

    # Nodo para robot_state_publisher con el remapeo adecuado
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        respawn=False,
        remappings=[('/joint_states', '/onrobot_2fg7/joint_states')],
        parameters=[{"robot_description": robot_description_content}],
    )

    return LaunchDescription([
        controller_spawner,
        robot_state_publisher,
    ])
