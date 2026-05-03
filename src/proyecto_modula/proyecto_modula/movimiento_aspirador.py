#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
import math

def euler_a_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class CortadorSeguro(Node):
    def __init__(self):
        super().__init__('cerebro_cortador_final')

        # --- AJUSTES DE SEGURIDAD ---
        self.map_msg = None
        self.margen_seguridad = 0.35  # Aumentado para evitar bordes y abortos
        self.ancho_corte = 0.10
        self.rutas = []
        self.punto_actual = 0

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_map)

        self.get_logger().info("1/4. Esperando mapa...")

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.get_logger().info("2/4. ¡Mapa recibido!")
        self.map_msg = msg
        
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 no responde.")
            return

        self.procesar_y_arrancar(msg)

    def es_punto_valido(self, x_world, y_world):
        if self.map_msg is None: return False
    
        res = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height
    
        # Convertir a coordenadas de celda
        grid_x = int((x_world - origin_x) / res)
        grid_y = int((y_world - origin_y) / res)
    
        # 1. Verificar límites estrictos del array
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
    
        # 2. Verificar un área pequeña (3x3 celdas) alrededor del punto
        # Esto asegura que el robot quepa y no esté pegado a una pared desconocida
        for i in range(-1, 2):
            for j in range(-1, 2):
                nx, ny = grid_x + i, grid_y + j
                if 0 <= nx < width and 0 <= ny < height:
                    index = ny * width + nx
                    # Si alguna celda cerca es pared (100) o desconocida (-1 o 255), no es válido
                    if self.map_msg.data[index] != 0:
                        return False
        return True

    def procesar_y_arrancar(self, msg):
        width = msg.info.width
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        min_x_idx, max_x_idx = width, 0
        min_y_idx, max_y_idx = msg.info.height, 0
        encontro_libre = False

        # Encontrar los límites de la zona libre (0)
        for i, val in enumerate(msg.data):
            if val == 0:
                encontro_libre = True
                y, x = divmod(i, width)
                min_x_idx, max_x_idx = min(min_x_idx, x), max(max_x_idx, x)
                min_y_idx, max_y_idx = min(min_y_idx, y), max(max_y_idx, y)

        if not encontro_libre:
            self.get_logger().error("No hay zonas libres en el mapa.")
            return

        # Aplicar margen de seguridad para no chocar con las paredes del mapa
        self.x_min = (min_x_idx * res) + origin_x + self.margen_seguridad
        self.x_max = (max_x_idx * res) + origin_x - self.margen_seguridad
        self.y_min = (min_y_idx * res) + origin_y + self.margen_seguridad
        self.y_max = (max_y_idx * res) + origin_y - self.margen_seguridad

        self.get_logger().info(f"Área segura: X[{self.x_min:.2f}, {self.x_max:.2f}] Y[{self.y_min:.2f}, {self.y_max:.2f}]")
        self.calcular_rutas()

    def calcular_rutas(self):
        y_actual = self.y_min
        ir_derecha = True
        paso_x = 0.30  # Distancia entre puntos de la misma línea (30cm)
    
        while y_actual <= self.y_max:
            # Generar una lista de puntos a lo largo de la línea X
            x_puntos = []
            curr_x = self.x_min
            while curr_x <= self.x_max:
                x_puntos.append(curr_x)
                curr_x += paso_x
            
            # Si vamos de regreso, invertimos la lista para el zigzag
            if not ir_derecha:
                x_puntos.reverse()
    
            for x in x_puntos:
                yaw = 0.0 if ir_derecha else 3.14
                if self.es_punto_valido(x, y_actual):
                    self.rutas.append((x, y_actual, yaw))
            
            y_actual += self.ancho_corte
            ir_derecha = not ir_derecha
        
    def ir_al_siguiente_punto(self):
        if self.punto_actual >= len(self.rutas):
            self.get_logger().info("¡Misión cumplida!")
            return

        x, y, yaw = self.rutas[self.punto_actual]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)

        self.get_logger().info(f"Yendo a: {x:.2f}, {y:.2f}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Punto rechazado por Nav2. Saltando al siguiente.")
            self.proximo_punto()
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Independientemente de si llegó o abortó, pasamos al siguiente para no quedar trabados
        self.proximo_punto()

    def proximo_punto(self):
        self.punto_actual += 1
        self.ir_al_siguiente_punto()

def main(args=None):
    rclpy.init(args=args)
    nodo = CerebroCortador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass # Ignora el error al presionar Ctrl+C
    finally:
        # Solo cierra si ROS sigue activo
        if rclpy.ok():
            nodo.destroy_node()
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
