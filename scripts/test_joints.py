#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FingerJointPublisher(Node):
    def __init__(self):
        super().__init__('finger_joint_publisher')
        # Publicador en un tópico (puedes usar /joint_states o uno diferente)
        self.publisher_ = self.create_publisher(JointState, 'finger_joint_states', 10)
        timer_period = 0.1  # Publica a 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_finger_link', 'left_finger_joint']
        # Asigna valores de posición; modifica según lo que necesites
        msg.position = [0.5, 0.5]
        # Si es necesario, asigna velocidades y esfuerzos
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publicado finger joint state')

def main(args=None):
    rclpy.init(args=args)
    node = FingerJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
