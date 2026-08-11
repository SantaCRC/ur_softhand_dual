import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import yaml
import time
import sys

# Velocidad del brazo: 0.05 = 5%, 0.10 = 10%, 0.20 = 20%, 1.0 = 100%
VELOCIDAD_BRAZO = 0.10   

GRUPO_BRAZO = "Left_arm"
PREFIJO_BRAZO = "ur_dual_D_"
PREFIJO_OTRO_BRAZO = "ur_dual_I_"
# ================================================================

class MoveUR5eAndHand(Node):
    def __init__(self):
        super().__init__('move_ur5e_and_hand')
        
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.hand_publisher = self.create_publisher(
            JointTrajectory,
            '/qbhand2m1/qbhand2m1_synergies_trajectory_controller/joint_trajectory',
            10
        )
        
        self.last_arm_position = None
        self.last_hand_position = None
        
        self.get_logger().info(f'Configuracion:')
        self.get_logger().info(f'  Grupo MoveIt: {GRUPO_BRAZO}')
        self.get_logger().info(f'  Prefijo joints: {PREFIJO_BRAZO}')
        self.get_logger().info(f'  Velocidad: {int(VELOCIDAD_BRAZO * 100)}%')
        self.get_logger().info(f'Esperando MoveGroup action server...')
        
        while not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Todavia esperando MoveGroup...')
        
        self.get_logger().info('MoveGroup conectado!')

    def is_arm_joint(self, name):
        return "ur5e" in name or "ur_dual" in name

    def create_motion_plan_request(self, joint_values, velocity_scale):
        request = MotionPlanRequest()
        request.group_name = GRUPO_BRAZO
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = velocity_scale

        joint_constraints = []
        for joint_name, joint_value in joint_values.items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = math.radians(joint_value)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            joint_constraints.append(constraint)

        request.goal_constraints.append(Constraints(joint_constraints=joint_constraints))
        return request

    def move_arm(self, joint_values, time_to_step, velocity_scale):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.create_motion_plan_request(joint_values, velocity_scale)
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.request.allowed_planning_time = 10.0
        
        joints_str = ", ".join(f"{k.replace(PREFIJO_BRAZO, '')}={v:.1f}°" 
                               for k, v in list(joint_values.items())[:3])
        self.get_logger().info(f"[BRAZO] Moviendo {GRUPO_BRAZO}: {joints_str}...")
        
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("[BRAZO] Movimiento RECHAZADO por MoveIt")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info(f"[BRAZO] OK")
            return True
        else:
            self.get_logger().error(f"[BRAZO] ERROR codigo: {result.result.error_code.val}")
            return False

    def move_hand(self, joint_values, time_to_step):
        hand_msg = JointTrajectory()
        
        # CORRECCIÓN 1: Estampar el mensaje con el reloj exacto de ROS2
        hand_msg.header.stamp = self.get_clock().now().to_msg()
        
        # CORRECCIÓN 2: Forzar el orden estricto de las sinergias
        nombres_mano = ['qbhand2m1_synergy_joint', 'qbhand2m1_manipulation_joint']
        hand_msg.joint_names = nombres_mano
        
        hand_point = JointTrajectoryPoint()
        # Mapear los valores de forma segura en el orden correcto
        hand_point.positions = [float(joint_values.get(name, 0.0)) for name in nombres_mano]
        
        # CORRECCIÓN 3: Casteo seguro a enteros para el tiempo
        hand_point.time_from_start.sec = int(time_to_step // 1000)
        hand_point.time_from_start.nanosec = int((time_to_step % 1000) * 1000000)
        hand_msg.points.append(hand_point)

        self.hand_publisher.publish(hand_msg)
        
        synergy = joint_values.get('qbhand2m1_synergy_joint', 0)
        estado = "CERRADA" if synergy > 0.3 else "ABIERTA"
        self.get_logger().info(f"[MANO] {estado} (synergy={synergy:.2f})")
        
        # Pausa activa para la mano
        self.safe_sleep(time_to_step / 1000.0)

    def safe_sleep(self, duration_sec):
        """Reemplazo de time.sleep() para no asfixiar el nodo de ROS2."""
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def has_position_changed(self, current_position, last_position):
        if last_position is None:
            return True
        tolerance_hand = 0.1
        return any(
            abs(current_position.get(joint, 0) - last_position.get(joint, 0)) > tolerance_hand
            if "qbhand" in joint else
            current_position.get(joint) != last_position.get(joint)
            for joint in current_position
        )

    def execute_trajectory_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                positions = data.get('positions', [])
                loop = data.get('loop', False)
            except yaml.YAMLError as exc:
                self.get_logger().error(f"Error al leer YAML: {exc}")
                return

        total = len(positions)
        self.get_logger().info(f'Cargadas {total} posiciones desde {yaml_path}')
        self.get_logger().info(f'Loop: {loop}')
        self.get_logger().info('='*50)

        while True:
            for i, position in enumerate(positions):
                joint_values = position['joint_values']
                time_to_step = position.get('time_to_step', 4000)
                delay_in_position = position.get('delay_in_position', 1000)

                self.get_logger().info(f'\n--- Paso {i+1}/{total} ---')

                arm_joint_values = {k: v for k, v in joint_values.items() if PREFIJO_BRAZO in k}
                hand_joint_values = {k: v for k, v in joint_values.items() if "qbhand" in k}

                if arm_joint_values and self.has_position_changed(arm_joint_values, self.last_arm_position):
                    success = self.move_arm(arm_joint_values, time_to_step, VELOCIDAD_BRAZO)
                    if success:
                        self.last_arm_position = arm_joint_values
                else:
                    self.get_logger().info("[BRAZO] Sin cambios, omitiendo.")

                if hand_joint_values and self.has_position_changed(hand_joint_values, self.last_hand_position):
                    self.move_hand(hand_joint_values, min(time_to_step, 2000))
                    self.last_hand_position = hand_joint_values
                else:
                    self.get_logger().info("[MANO] Sin cambios, omitiendo.")

                if delay_in_position > 0:
                    self.get_logger().info(f"Esperando {delay_in_position}ms...")
                    self.safe_sleep(delay_in_position / 1000.0)

            if not loop:
                break

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('SECUENCIA COMPLETADA')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5eAndHand()

    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]
    else:
        yaml_path = input("Ruta del archivo YAML: ")
    
    node.execute_trajectory_from_yaml(yaml_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()