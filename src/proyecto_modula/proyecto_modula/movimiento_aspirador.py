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
        
        # --- CONFIGURACIÓN ---
        self.tiempo_limite = 30.0  
        self.margen_seguridad = 0.55
        self.ancho_corte = 0.40
        self.paso_puntos = 0.60
        
        self.rutas = []
        self.punto_actual = 0
        self.map_msg = None
        self.goal_handle = None
        self.inicio_tiempo_punto = None
        
        self.esperando_meta = False
        self.sistema_listo = False

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        qos_map = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, 
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        self.timer_watchdog = self.create_timer(1.0, self.revisar_progreso)
        self.get_logger().info("1/4. Esperando mapa...")

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.map_msg = msg
        self.get_logger().info("2/4. ¡Mapa recibido!")
        self.verificar_nav2()

    def verificar_nav2(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Esperando a Nav2...")
            self.create_timer(2.0, self.verificar_nav2)
            return
        
        if not self.sistema_listo:
            self.sistema_listo = True
            self.get_logger().info("3/4. Nav2 detectado.")
            self.generar_zigzag_completo()

    def generar_zigzag_completo(self):
        # Lógica original para calcular el área y los puntos
        msg = self.map_msg
        width, res = msg.info.width, msg.info.resolution
        origin_x, origin_y = msg.info.origin.position.x, msg.info.origin.position.y
        
        min_x_idx, max_x_idx = width, 0
        min_y_idx, max_y_idx = msg.info.height, 0
        
        for i, val in enumerate(msg.data):
            if val == 0:
                y, x = divmod(i, width)
                min_x_idx, max_x_idx = min(min_x_idx, x), max(max_x_idx, x)
                min_y_idx, max_y_idx = min(min_y_idx, y), max(max_y_idx, y)

        x_min = (min_x_idx * res) + origin_x + self.margen_seguridad
        x_max = (max_x_idx * res) + origin_x - self.margen_seguridad
        y_min = (min_y_idx * res) + origin_y + self.margen_seguridad
        y_max = (max_y_idx * res) + origin_y - self.margen_seguridad

        # Generar la lista de puntos
        y_curr = y_min
        derecha = True
        while y_curr <= y_max:
            linea = []
            x_curr = x_min
            while x_curr <= x_max:
                linea.append(x_curr)
                x_curr += self.paso_puntos
            if not derecha: linea.reverse()
            for x in linea:
                yaw = 0.0 if derecha else 3.1416
                self.rutas.append((x, y_curr, yaw))
            y_curr += self.ancho_corte
            derecha = not derecha

        self.get_logger().info(f"4/4. {len(self.rutas)} rutas generadas. Iniciando en 5s...")
        self.timer_inicio = self.create_timer(5.0, self.iniciar_mision)

    def iniciar_mision(self):
        self.timer_inicio.cancel()
        self.ir_al_siguiente_punto()

    def ir_al_siguiente_punto(self):
        if self.punto_actual >= len(self.rutas) or self.esperando_meta:
            return

        x, y, yaw = self.rutas[self.punto_actual]
        self.get_logger().info(f"==> Punto {self.punto_actual+1}/{len(self.rutas)}")
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = euler_a_quaternion(yaw)

        self.esperando_meta = True
        self.inicio_tiempo_punto = self.get_clock().now()
        
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.esperando_meta = False
            self.proximo_con_pausa()
            return
        self.goal_handle = handle
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.esperando_meta:
            self.proximo_con_pausa()

    def revisar_progreso(self):
        if not self.esperando_meta or not self.inicio_tiempo_punto:
            return
            
        t = (self.get_clock().now() - self.inicio_tiempo_punto).nanoseconds / 1e9
        if t >= self.tiempo_limite:
            self.get_logger().warn(f"TIMEOUT en Punto {self.punto_actual+1}")
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.proximo_con_pausa()

    def proximo_con_pausa(self):
        if not self.esperando_meta: return # Evitar doble llamada
        self.esperando_meta = False
        self.punto_actual += 1
        self.goal_handle = None
        # Pausa de 2s para limpiar Status 6
        self.timer_p = self.create_timer(2.0, self.disparar_siguiente)

    def disparar_siguiente(self):
        self.timer_p.cancel()
        self.ir_al_siguiente_punto()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CortadorSeguro())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
