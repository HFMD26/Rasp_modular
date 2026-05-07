#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion
import math

class LineaRecta(Node):
    def __init__(self):
        super().__init__('test_linea_recta')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Puntos: (x, y, yaw)
        # 1. Ir 2 metros adelante. 2. Volver al inicio.
        self.puntos = [(2.0, 0.0, 0.0), (0.0, 0.0, 3.1416)]
        self.indice = 0
        
        self.get_logger().info("Esperando a Nav2...")
        self.nav_client.wait_for_server()
        self.enviar_meta()

    def enviar_meta(self):
        if self.indice >= len(self.puntos):
            self.get_logger().info("Prueba de línea recta terminada.")
            return

        x, y, yaw = self.puntos[self.indice]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        
        # Convertir yaw a Quaternion
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)

        self.get_logger().info(f"Yendo a: x={x}, y={y}")
        self.nav_client.send_goal_async(goal).add_done_callback(self.resultado_callback)

    def resultado_callback(self, future):
        handle = future.result()
        handle.get_result_async().add_done_callback(self.fin_callback)

    def fin_callback(self, future):
        self.get_logger().info("Punto alcanzado.")
        self.indice += 1
        self.enviar_meta()

def main():
    rclpy.init()
    rclpy.spin(LineaRecta())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
