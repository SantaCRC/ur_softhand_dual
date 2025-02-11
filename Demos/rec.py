import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os
import time
import math

class PositionRecorder(Node):
    def __init__(self):
        super().__init__('position_recorder')
        
        # Suscriptor para escuchar las posiciones actuales de las juntas
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Variables para almacenar las posiciones de las juntas y la última posición guardada
        self.current_positions = {}
        self.last_saved_positions = {}
        
        # Variable para verificar si se ha recibido algún dato
        self.received_data = False

    def joint_state_callback(self, msg):
        # Actualizar las posiciones actuales en el orden recibido
        self.current_positions = {name: position for name, position in zip(msg.name, msg.position)}
        
        # Marcar que se han recibido datos
        self.received_data = True

    def print_current_positions(self):
        # Función para imprimir las posiciones actuales en la consola, en grados
        print("\nPosiciones actuales de las juntas (en grados):")
        for name, position in self.current_positions.items():
            if "qbhand" in name:  # Las posiciones de la mano no se convierten a grados
                print(f"{name}: {round(position, 3)}")
            else:
                position_degrees = math.degrees(position)
                print(f"{name}: {round(position_degrees, 3)}°")

    def has_position_changed(self):
        # Verifica si la posición actual es diferente de la última posición guardada, excluyendo las juntas qbhand2m1_motor_1_joint y qbhand2m1_motor_2_joint
        return self.last_saved_positions != {
            name: (round(math.degrees(value), 3) if "ur5e" in name else round(value, 3))
            for name, value in self.current_positions.items()
            if name not in ["qbhand2m1_motor_1_joint", "qbhand2m1_motor_2_joint"]
        }

    def save_position(self, file_path):
        # Llama al tópico para obtener los valores actuales
        rclpy.spin_once(self, timeout_sec=1.0)

        # Imprimir los valores actuales para verificación
        self.print_current_positions()
        
        # Guardar la posición actual en el archivo YAML solo si ha cambiado
        if self.received_data and self.current_positions and self.has_position_changed():
            # Convertir cada posición a grados solo para el brazo antes de guardar, excluyendo las juntas qbhand2m1_motor_1_joint y qbhand2m1_motor_2_joint
            position_entry = {
                'joint_values': {
                    name: (round(math.degrees(value), 3) if "ur5e" in name else round(value, 3))
                    for name, value in self.current_positions.items()
                    if name not in ["qbhand2m1_motor_1_joint", "qbhand2m1_motor_2_joint"]
                },
                'time_to_step': 4000,
                'delay_in_position': 1000
            }

            # Cargar el archivo existente o crear una nueva estructura de datos
            if os.path.exists(file_path):
                with open(file_path, 'r') as file:
                    data = yaml.safe_load(file) or {}
            else:
                data = {'positions': []}

            # Agregar la nueva posición al archivo
            data['positions'].append(position_entry)

            # Guardar el archivo actualizado
            with open(file_path, 'w') as file:
                yaml.dump(data, file)
            
            self.last_saved_positions = position_entry['joint_values']  # Actualizar la última posición guardada
            self.get_logger().info(f"Posición guardada en {file_path}.")
        else:
            self.get_logger().info("No se han detectado cambios en las posiciones de las juntas.")

def main(args=None):
    rclpy.init(args=args)
    node = PositionRecorder()
    
    # Esperar hasta recibir datos al menos una vez
    print("Esperando a recibir datos en /joint_states...")
    while not node.received_data:
        rclpy.spin_once(node, timeout_sec=1.0)
    
    print("Datos recibidos. Puede empezar a guardar posiciones.")

    file_path = input("Por favor ingrese la ruta y el nombre del archivo YAML para guardar las posiciones: ")

    # Ciclo para guardar múltiples posiciones
    try:
        while True:
            input("Presione Enter para guardar la posición actual o Ctrl+C para salir...")
            node.save_position(file_path)
    except KeyboardInterrupt:
        print("\nGuardado de posiciones finalizado.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
