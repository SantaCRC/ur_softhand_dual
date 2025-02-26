import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class URScriptPublisher(Node):
    def __init__(self):
        super().__init__('urscript_publisher')
        self.publisher_ = self.create_publisher(String, 'ur_dual_D/urscript_interface/script_command', 10)
        self.timer = self.create_timer(1.0, self.publish_script)  # Publica después de 1 segundo

    def publish_script(self):
        msg = String()
        msg.data = """sec my_prog():

  textmsg("Hola")

end"""
        self.publisher_.publish(msg)
        self.get_logger().info('Script enviado al URScript Interface')

        # Cancela el temporizador para que no se ejecute más veces
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = URScriptPublisher()
    
    try:
        rclpy.spin(node)  # Mantiene el nodo activo hasta que se publique el mensaje
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
