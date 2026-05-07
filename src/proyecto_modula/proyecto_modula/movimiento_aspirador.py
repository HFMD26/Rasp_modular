#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
from rclpy.duration import Duration
import math
import time

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
        
        # --- CONFIGURACIÓN (Robot 75.5cm x 46cm) ---
        self.margen_seguridad = 0.55
        self.ancho_corte = 0.40
        self.paso_puntos = 0.60
        
        self.map_msg = None
        self.rutas = []
        self.punto_actual = 0
        self.goal_handle = None
        self.inicio_tiempo_punto = None
        self.esperando_meta = False  # CANDADO DE SEGURIDAD
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        # Timer de revisión cada 2 segundos
        self.timer_revision = self.create_timer(2.0, self.revisar_progreso)
        
        self.get_logger().info("Sistema iniciado. Esperando mapa...")

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.map_msg = msg
        self.procesar_y_arrancar(msg)

    def es_punto_valido(self, x_world, y_world):
        if self.map_msg is None: return False
        res = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        grid_x = int((x_world - origin_x) / res)
        grid_y = int((y_world - origin_y) / res)
        index = grid_y * self.map_msg.info.width + grid_x
        return self.map_msg.data[index] == 0

    def procesar_y_arrancar(self, msg):
        width = msg.info.width
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        min_x_idx, max_x_idx = width, 0
        min_y_idx, max_y_idx = msg.info.height, 0
        for i, val in enumerate(msg.data):
            if val == 0:
                y, x = divmod(i, width)
                min_x_idx, max_x_idx = min(min_x_idx, x), max(max_x_idx, x)
                min_y_idx, max_y_idx = min(min_y_idx, y), max(max_y_idx, y)

        self.x_min = (min_x_idx * res) + origin_x + self.margen_seguridad
        self.x_max = (max_x_idx * res) + origin_x - self.margen_seguridad
        self.y_min = (min_y_idx * res) + origin_y + self.margen_seguridad
        self.y_max = (max_y_idx * res) + origin_y - self.margen_seguridad

        self.generar_zigzag()
        if self.rutas:
            self.ir_al_siguiente_punto()

    def generar_zigzag(self):
        y_actual = self.y_min
        hacia_derecha = True
        while y_actual <= self.y_max:
            linea = []
            x_actual = self.x_min
            while x_actual <= self.x_max:
                linea.append(x_actual)
                x_actual += self.paso_puntos
            if not hacia_derecha: linea.reverse()
            for x in linea:
                yaw = 0.0 if hacia_derecha else 3.1416
                if self.es_punto_valido(x, y_actual):
                    self.rutas.append((x, y_actual, yaw))
            y_actual += self.ancho_corte
            hacia_derecha = not hacia_derecha

    def ir_al_siguiente_punto(self):
        if self.punto_actual >= len(self.rutas):
            self.get_logger().info("¡TRABAJO TERMINADO!")
            return
            
        x, y, yaw = self.rutas[self.punto_actual]
        
        # --- RESET TOTAL DE ESTADO PARA EL NUEVO PUNTO ---
        self.esperando_meta = True 
        self.goal_handle = None # Limpiamos el handle anterior
        self.inicio_tiempo_punto = self.get_clock().now() 
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)
        
        self.get_logger().info(f"Yendo a punto {self.punto_actual+1}/{len(self.rutas)}...")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.proximo_punto()
            return
        self.goal_handle = goal_handle
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Si ya no estamos esperando (porque el timeout actuó antes), ignoramos este resultado
        if not self.esperando_meta:
            return
            
        self.get_logger().info("¡Punto alcanzado!")
        self.esperando_meta = False
        self.proximo_punto()

    def revisar_progreso(self):
        # IMPORTANTE: Solo checar si estamos esperando activamente
        if not self.esperando_meta or self.inicio_tiempo_punto is None:
            return
            
        transcurrido = self.get_clock().now() - self.inicio_tiempo_punto
        
        # 45 segundos de tiempo máximo
        if transcurrido > Duration(seconds=45):
            self.get_logger().warn(f"TIMEOUT. Saltando punto {self.punto_actual+1}")
            
            # BLOQUEO INMEDIATO para evitar que get_result_callback se ejecute
            self.esperando_meta = False 
            self.inicio_tiempo_punto = None
            
            # Cancelar en Nav2 si hay un handle
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            
            self.proximo_punto()

    def proximo_punto(self):
        self.punto_actual += 1
        # Pequeña pausa real para que los hilos de ROS se sincronicen
        time.sleep(0.2) 
        self.ir_al_siguiente_punto()

def main(args=None):
    rclpy.init(args=args)
    nodo = CortadorSeguro()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
