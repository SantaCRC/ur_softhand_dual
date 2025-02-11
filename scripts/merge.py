#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # Subscripciones para cada tópico de estados de articulaciones
        self.joint_ur5_sub = self.create_subscription(
            JointState, '/joint_states', self.ur5_callback, 10)
        self.joint_qbhand_sub = self.create_subscription(
            JointState, '/qbhand2m1/joint_states', self.qbhand_callback, 10)

        # Publicador en el tópico original /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Variables para almacenar los estados de articulaciones
        self.ur5_joint_state = JointState()
        self.qbhand_joint_state = JointState()

    def ur5_callback(self, msg):
        self.ur5_joint_state = msg
        self.publish_merged_joint_states()

    def qbhand_callback(self, msg):
        self.qbhand_joint_state = msg
        self.publish_merged_joint_states()

    def publish_merged_joint_states(self):
        # Crear el mensaje combinado
        merged_msg = JointState()

        # Combinar nombres, posiciones, velocidades y esfuerzos de ambas fuentes
        merged_msg.name = self.ur5_joint_state.name + self._replace_qbhand_names(self.qbhand_joint_state.name)
        merged_msg.position = list(self.ur5_joint_state.position) + list(self.qbhand_joint_state.position)
        merged_msg.velocity = list(self.ur5_joint_state.velocity) + list(self.qbhand_joint_state.velocity)
        merged_msg.effort = list(self.ur5_joint_state.effort) + list(self.qbhand_joint_state.effort)

        # Establecer la marca de tiempo
        merged_msg.header.stamp = self.get_clock().now().to_msg()

        # Publicar el mensaje combinado en /joint_states
        self.publisher_.publish(merged_msg)

    def _replace_qbhand_names(self, qbhand_names):
        # Remueve el prefijo "m1" de los nombres de las articulaciones de QBHand
        return [name.replace("qbhand2m1_", "qbhand2m1_") for name in qbhand_names]

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
