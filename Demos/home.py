import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import yaml
import time

class MoveUR5eAndHand(Node):
    def __init__(self):
        super().__init__('move_ur5e_and_hand')
        
        # Cliente de acción de MoveGroup para el brazo
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Publicador para el controlador de trayectoria de la mano
        self.hand_publisher = self.create_publisher(JointTrajectory, '/qbhand2m1/qbhand2m1_synergies_trajectory_controller/joint_trajectory', 10)
        
        # Variables para almacenar la última posición
        self.last_arm_position = None
        self.last_hand_position = None
        
        # Esperar a que el cliente de acción esté disponible
        while not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el cliente de MoveGroup esté disponible...')

    def create_motion_plan_request(self, joint_values, group_name, velocity_scale=0.2, acceleration_scale=0.2):
        # Crear solicitud de planificación de movimiento para el brazo
        request = MotionPlanRequest()
        request.group_name = group_name
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = acceleration_scale

        joint_constraints = []
        for joint_name, joint_value in joint_values.items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = math.radians(joint_value)
            constraint.tolerance_above = 0.01 if "ur5e" in joint_name else 0.05
            constraint.tolerance_below = constraint.tolerance_above
            constraint.weight = 1.0
            joint_constraints.append(constraint)

        request.goal_constraints.append(Constraints(joint_constraints=joint_constraints))
        return request

    def move_to_position(self, joint_values, time_to_step, group_name, velocity_scale):
        if group_name == "Hand":
            # Publicar directamente en el tópico de la mano
            hand_msg = JointTrajectory()
            hand_msg.joint_names = list(joint_values.keys())
            
            hand_point = JointTrajectoryPoint()
            hand_point.positions = [joint_values[name] for name in hand_msg.joint_names]
            hand_point.time_from_start.sec = time_to_step // 1000  # Convertir milisegundos a segundos
            hand_point.time_from_start.nanosec = (time_to_step % 1000) * 1000000  # Resto en nanosegundos
            hand_msg.points.append(hand_point)

            # Publicar la trayectoria de la mano
            self.hand_publisher.publish(hand_msg)
            self.get_logger().info(f"Enviado comando directo a la mano en {hand_msg.joint_names} con posiciones {hand_point.positions}.")
            time.sleep(time_to_step / 1000.0)  # Espera para la siguiente posición
            return

        # Crear una solicitud de movimiento para el brazo con MoveGroup
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.create_motion_plan_request(joint_values, group_name, velocity_scale=velocity_scale, acceleration_scale=velocity_scale)
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.request.allowed_planning_time = 5.0
        
        self.get_logger().info(f"Enviando solicitud de movimiento a MoveGroup para el grupo {group_name}...")
        future = self.move_group_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("La solicitud de movimiento fue rechazada.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info(f"Movimiento del grupo {group_name} completado exitosamente.")
            time.sleep(time_to_step / 1000.0)
        else:
            self.get_logger().error(f"Error en la ejecución del movimiento del grupo {group_name}. Código de error: {result.result.error_code.val}")

    def has_position_changed(self, current_position, last_position):
        tolerance_hand = 0.1
        return last_position is None or any(
            abs(current_position.get(joint, 0) - last_position.get(joint, 0)) > tolerance_hand if "qbhand" in joint else
            current_position.get(joint) != last_position.get(joint)
            for joint in current_position
        )

    def execute_trajectory_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                positions = data.get('positions', [])
                loop = data.get('loop', False)
                hand_speed = data.get('hand_speed', 500) / 100.0
            except yaml.YAMLError as exc:
                self.get_logger().error(f"Error al leer el archivo YAML: {exc}")
                return

        while True:
            for position in positions:
                joint_values = position['joint_values']
                time_to_step = position.get('time_to_step', 4000)
                delay_in_position = position.get('delay_in_position', 1000)

                arm_joint_values = {k: v for k, v in joint_values.items() if "ur5e" in k}
                hand_joint_values = {k: v for k, v in joint_values.items() if "qbhand" in k}

                # Mover el brazo solo si hay un cambio de posición
                if arm_joint_values and self.has_position_changed(arm_joint_values, self.last_arm_position):
                    self.move_to_position(arm_joint_values, time_to_step, group_name="Arm", velocity_scale=0.2)
                    self.last_arm_position = arm_joint_values
                else:
                    self.get_logger().info("No se detectaron cambios en la posición del brazo, omitiendo movimiento.")

                # Mover la mano solo si hay un cambio de posición dentro del rango de 0.1
                if hand_joint_values and self.has_position_changed(hand_joint_values, self.last_hand_position):
                    self.move_to_position(hand_joint_values, time_to_step, group_name="Hand", velocity_scale=hand_speed)
                    self.last_hand_position = hand_joint_values
                else:
                    self.get_logger().info("No se detectaron cambios significativos en la posición de la mano, omitiendo movimiento.")

                # Esperar solo si se realizó algún movimiento
                if (arm_joint_values and self.has_position_changed(arm_joint_values, self.last_arm_position)) or \
                   (hand_joint_values and self.has_position_changed(hand_joint_values, self.last_hand_position)):
                    self.get_logger().info(f"Esperando {delay_in_position} ms en la posición actual.")
                    time.sleep(delay_in_position / 1000.0)

            if not loop:
                break

def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5eAndHand()

    yaml_path = input("Por favor ingrese la ruta del archivo YAML de posiciones: ")
    node.execute_trajectory_from_yaml(yaml_path)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
