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
        
        # --- CONFIGURACIÓN GEOMÉTRICA (Robot 75.5cm x 46cm) ---
        self.margen_seguridad = 0.55  # Margen para que el frente (41cm) no choque
        self.ancho_corte = 0.40      # Distancia entre líneas de zigzag
        self.paso_puntos = 0.60      # Distancia entre puntos de la misma línea
        
        # --- ESTADOS Y VARIABLES ---
        self.map_msg = None
        self.rutas = []
        self.punto_actual = 0
        self.goal_handle = None
        self.inicio_tiempo_punto = None
        self.esperando_meta = False  # Candado para evitar cascada de puntos
        
        # --- CLIENTE DE ACCIÓN ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- SUBSCRIPCIÓN AL MAPA ---
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        # --- TIMER DE SEGURIDAD (WATCHDOG) ---
        # Revisa cada 2 segundos si el robot se quedó atorado
        self.timer_revision = self.create_timer(2.0, self.revisar_progreso)
        
        self.get_logger().info("1/4. Sistema iniciado. Esperando mapa...")

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.get_logger().info("2/4. ¡Mapa recibido! Procesando área...")
        self.map_msg = msg
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 no responde. ¿Está prendido el simulador?")
            return
        self.procesar_y_arrancar(msg)

    def es_punto_valido(self, x_world, y_world):
        if self.map_msg is None: return False
        res = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        
        grid_x = int((x_world - origin_x) / res)
        grid_y = int((y_world - origin_y) / res)
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
        index = grid_y * width + grid_x
        # 0 es espacio libre en OccupancyGrid
        return self.map_msg.data[index] == 0

    def procesar_y_arrancar(self, msg):
        width = msg.info.width
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        min_x_idx, max_x_idx = width, 0
        min_y_idx, max_y_idx = msg.info.height, 0
        encontro_libre = False

        for i, val in enumerate(msg.data):
            if val == 0:
                encontro_libre = True
                y, x = divmod(i, width)
                min_x_idx, max_x_idx = min(min_x_idx, x), max(max_x_idx, x)
                min_y_idx, max_y_idx = min(min_y_idx, y), max(max_y_idx, y)

        if not encontro_libre:
            self.get_logger().error("No se encontraron zonas libres en el mapa.")
            return

        # Calcular límites con margen de seguridad
        self.x_min = (min_x_idx * res) + origin_x + self.margen_seguridad
        self.x_max = (max_x_idx * res) + origin_x - self.margen_seguridad
        self.y_min = (min_y_idx * res) + origin_y + self.margen_seguridad
        self.y_max = (max_y_idx * res) + origin_y - self.margen_seguridad

        self.get_logger().info(f"3/4. Área segura: X[{self.x_min:.2f} : {self.x_max:.2f}]")
        self.generar_zigzag()
        
        if self.rutas:
            self.get_logger().info(f"4/4. Iniciando misión con {len(self.rutas)} puntos.")
            self.ir_al_siguiente_punto()

    def generar_zigzag(self):
        y_actual = self.y_min
        hacia_derecha = True
        
        while y_actual <= self.y_max:
            linea_puntos = []
            x_actual = self.x_min
            while x_actual <= self.x_max:
                linea_puntos.append(x_actual)
                x_actual += self.paso_puntos
            
            if not hacia_derecha: linea_puntos.reverse()
            
            for x in linea_puntos:
                yaw = 0.0 if hacia_derecha else 3.1416
                if self.es_punto_valido(x, y_actual):
                    self.rutas.append((x, y_actual, yaw))
            
            y_actual += self.ancho_corte
            hacia_derecha = not hacia_derecha

    def ir_al_siguiente_punto(self):
        if self.punto_actual >= len(self.rutas):
            self.get_logger().info("¡MISIÓN FINALIZADA EXITOSAMENTE!")
            self.esperando_meta = False
            self.inicio_tiempo_punto = None
            return
            
        x, y, yaw = self.rutas[self.punto_actual]
        
        # Reiniciar candado y reloj para el nuevo punto
        self.esperando_meta = True 
        self.inicio_tiempo_punto = self.get_clock().now()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)
        
        self.get_logger().info(f"Punto {self.punto_actual+1}/{len(self.rutas)} -> ({x:.2f}, {y:.2f})")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Nav2 rechazó el punto. Saltando...")
            self.esperando_meta = False
            self.proximo_punto()
            return
        
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Solo avanzar si el timeout no ganó la carrera
        if self.esperando_meta:
            self.get_logger().info("Punto alcanzado.")
            self.esperando_meta = False
            self.inicio_tiempo_punto = None
            self.proximo_punto()

    def revisar_progreso(self):
        # El watchdog solo actúa si hay una meta activa
        if not self.esperando_meta or self.inicio_tiempo_punto is None:
            return
            
        transcurrido = self.get_clock().now() - self.inicio_tiempo_punto
        
        # 50 segundos para considerar que se atoró
        if transcurrido > Duration(seconds=50):
            self.get_logger().warn(f"TIMEOUT en punto {self.punto_actual+1}. Saltando punto...")
            
            # 1. Bloquear callbacks futuros de este punto
            self.esperando_meta = False
            self.inicio_tiempo_punto = None 
            
            # 2. Intentar cancelar en Nav2
            if self.goal_handle:
                self.nav_client.cancel_goal_async(self.goal_handle)
            
            # 3. Moverse al siguiente
            self.proximo_punto()

    def proximo_punto(self):
        self.punto_actual += 1
        # Pequeña pausa de seguridad antes de la siguiente iteración
        self.ir_al_siguiente_punto()

def main(args=None):
    rclpy.init(args=args)
    nodo = CortadorSeguro()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Deteniendo por usuario...")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
