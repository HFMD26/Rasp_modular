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
        self.tiempo_limite = 30.0  
        self.margen_seguridad = 0.55
        self.ancho_corte = 0.40
        self.paso_puntos = 0.60
        
        self.rutas = []
        self.punto_actual = 0
        self.map_msg = None
        self.goal_handle = None
        self.inicio_tiempo_punto = None
        
        # ESTADOS DE CONTROL (Para evitar duplicados)
        self.esperando_meta = False
        self.sistema_listo = False

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        qos_map = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, 
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        self.timer_watchdog = self.create_timer(1.0, self.revisar_progreso)
        
        self.get_logger().info("1/4. Esperando mapa y sistema listos...")

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.map_msg = msg
        self.get_logger().info("2/4. ¡Mapa recibido!")
        
        # ESPERA ACTIVA: No procesamos hasta que Nav2 responda
        self.create_timer(2.0, self.verificar_nav2)

    def verificar_nav2(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Esperando a que Nav2 despierte...")
            return
        
        if not self.sistema_listo:
            self.sistema_listo = True
            self.get_logger().info("3/4. Nav2 detectado.")
            self.procesar_y_arrancar()

    def procesar_y_arrancar(self):
        # Lógica de generación de zigzag (Simplificada para el ejemplo)
        # Aquí calculas tus 83 u 88 puntos basados en self.map_msg
        self.get_logger().info("4/4. Rutas generadas. Iniciando en 5 segundos para permitir ubicación...")
        
        # Generar puntos (zigzag)...
        # self.rutas.append(...) 
        
        # Pausa de seguridad para que AMCL ubique al robot antes del primer punto
        self.create_timer(5.0, self.ir_al_siguiente_punto)

    def ir_al_siguiente_punto(self):
        # PROTECCIÓN CRÍTICA: Si ya hay una meta en curso, NO enviar otra
        if self.punto_actual >= len(self.rutas) or self.esperando_meta:
            return

        x, y, yaw = self.rutas[self.punto_actual]
        self.get_logger().info(f"Enviando Punto {self.punto_actual+1}/{len(self.rutas)} -> ({x:.2f}, {y:.2f})")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)

        self.esperando_meta = True
        self.inicio_tiempo_punto = self.get_clock().now()
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Meta rechazada. Reintentando en breve...")
            self.esperando_meta = False
            return
        self.goal_handle = handle
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Punto {self.punto_actual+1} finalizado.")
        self.esperando_meta = False
        self.punto_actual += 1
        # Pausa de 2s para evitar el error de Status 6
        self.create_timer(2.0, self.ir_al_siguiente_punto)

    def revisar_progreso(self):
        if not self.esperando_meta or self.inicio_tiempo_punto is None:
            return
            
        transcurrido = (self.get_clock().now() - self.inicio_tiempo_punto).nanoseconds / 1e9
        
        if transcurrido >= self.tiempo_limite:
            self.get_logger().warn(f"¡TIEMPO AGOTADO! Saltando punto...")
            self.esperando_meta = False 
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.punto_actual += 1
            self.create_timer(2.0, self.ir_al_siguiente_punto)

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
