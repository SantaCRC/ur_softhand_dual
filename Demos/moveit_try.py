import rclpy
from rclpy.logging import get_logger
from moveit import MoveItPy
from geometry_msgs.msg import PoseStamped
def main():
   rclpy.init()
   logger = get_logger("ur5e_python_mover")

   try:
       ur5e_robot = MoveItPy(node_name="ur5e_python_mover")
   except Exception as e:
       logger.error(f"Nelze inicializovat MoveItPy. Spouštíte to přes launch file s parametry robota? Chyba: {e}")
       return

   ur5e_arm = ur5e_robot.get_planning_component("left_arm")
   # Nastavení počáteční konfigurace (volitelné, pro jistotu)
   ur5e_arm.set_start_state_to_current_state()
   # 3. Definice cílové pozice (PoseStamped)
   target_pose = PoseStamped()
   target_pose.header.frame_id = "ur_dual_I_tool0"  # Nebo "base" dle vaší konfigurace
   target_pose.header.stamp = ur5e_robot.get_clock().now().to_msg()
   # Pozice [metry]
   target_pose.pose.position.x = 0.502
   target_pose.pose.position.y = 0.45
   target_pose.pose.position.z = 1.0
   # Orientace [kvaterniony]
   # (zde orientace w=1.0, což může být pro UR5e nepřirozené, lepší je použít reálnou orientaci)
   target_pose.pose.orientation.w = 0.7
   target_pose.pose.orientation.x = -0.7
   target_pose.pose.orientation.y = 0.0
   target_pose.pose.orientation.z = 0.0
   # 4. Nastavení cíle
   ur5e_arm.set_goal_state(pose_stamped_msg=target_pose)
   # 5. Plánování
   logger.info("Zahajuji plánování trasy...")
   plan_result = ur5e_arm.plan()
   if plan_result:
       logger.info("Plán nalezen! Vykonávám pohyb...")
       # 6. Exekuce
       # MoveItPy pošle trajektorii controllerům
       ur5e_robot.execute(plan_result.trajectory, controllers=[])
       logger.info("Pohyb dokončen.")
   else:
       logger.error("Plánování selhalo. Zkontrolujte, zda je cíl v dosahu a bez kolizí.")
   rclpy.shutdown()
if __name__ == "__main__":
   main()