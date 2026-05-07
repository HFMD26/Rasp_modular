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
        
        # --- CONFIGURACIÓN ---
        self.tiempo_limite = 30.0  # 30 segundos reales
        self.margen_seguridad = 0.55
        self.ancho_corte = 0.40
        self.paso_puntos = 0.60
        
        # --- ESTADOS ---
        self.map_msg = None
        self.rutas = []
        self.punto_actual = 0
        self.goal_handle = None
        self.inicio_tiempo_punto = None
        self.esperando_meta = False
        self.bloqueo_pausa = False

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        qos_map = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, 
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        # Revisión de Watchdog cada 1 segundo
        self.timer_watchdog = self.create_timer(1.0, self.revisar_progreso)
        
        self.get_logger().info("Nodo iniciado. Esperando mapa para generar rutas...")

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.map_msg = msg
        self.procesar_y_arrancar(msg)

    def procesar_y_arrancar(self, msg):
        # ... (Lógica de zigzag igual a la anterior)
        width, res = msg.info.width, msg.info.resolution
        origin_x, origin_y = msg.info.origin.position.x, msg.info.origin.position.y
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
                self.rutas.append((x, y_actual, yaw))
            y_actual += self.ancho_corte
            hacia_derecha = not hacia_derecha
        self.get_logger().info(f"Rutas generadas: {len(self.rutas)} puntos.")

    def ir_al_siguiente_punto(self):
        if self.punto_actual >= len(self.rutas) or self.bloqueo_pausa:
            return

        x, y, yaw = self.rutas[self.punto_actual]
        self.get_logger().info(f"Enviando Punto {self.punto_actual+1}/{len(self.rutas)}")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)

        self.inicio_tiempo_punto = self.get_clock().now()
        self.esperando_meta = True
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Meta rechazada.")
            self.preparar_siguiente(0.5)
            return
        self.goal_handle = handle
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.esperando_meta:
            self.get_logger().info(f"Punto {self.punto_actual+1} terminado.")
            self.preparar_siguiente(2.0)

    def revisar_progreso(self):
        if not self.esperando_meta or self.inicio_tiempo_punto is None:
            return
            
        transcurrido = (self.get_clock().now() - self.inicio_tiempo_punto).nanoseconds / 1e9
        
        if transcurrido >= self.tiempo_limite:
            self.get_logger().warn(f"TIMEOUT: {transcurrido:.1f}s en Punto {self.punto_actual+1}")
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.preparar_siguiente(2.0)

    def preparar_siguiente(self, pausa):
        if self.bloqueo_pausa: return
        self.bloqueo_pausa = True
        self.esperando_meta = False
        self.punto_actual += 1
        self.goal_handle = None
        
        # Timer de un solo disparo para la pausa
        self.create_timer(pausa, self.finalizar_pausa)

    def finalizar_pausa(self):
        # Necesitamos encontrar y detener el timer que nos llamó para que no se repita
        # En ROS 2 Humble esto es un poco manual si no guardamos la referencia, 
        # así que usaremos un truco más limpio:
        self.bloqueo_pausa = False
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
