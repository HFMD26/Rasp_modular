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
        
        # --- NUEVAS MEDIDAS COHERENTES CON ROBOT 75.5 x 46 ---
        # El frente mide 0.41m. Añadimos un margen extra de 15cm para maniobra.
        self.margen_seguridad = 0.55  
        # El ancho es 0.46m. Pasadas de 0.40m aseguran cobertura total.
        self.ancho_corte = 0.40 
        
        self.map_msg = None
        self.rutas = []
        self.punto_actual = 0
        self.goal_handle = None
        self.inicio_tiempo_punto = None
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        # Timer para revisar si el robot se queda atorado (Cada 2 segundos)
        self.timer_revision = self.create_timer(2.0, self.revisar_progreso)
        
        self.get_logger().info("1/4. Esperando mapa y sistema listos...")

    def revisar_progreso(self):
        """Si el robot tarda más de 60s en un solo punto, saltamos al siguiente."""
        if self.inicio_tiempo_punto is None:
            return
            
        tiempo_transcurrido = self.get_clock().now() - self.inicio_tiempo_punto
        if tiempo_transcurrido > Duration(seconds=15):
            self.get_logger().warn("¡TIEMPO AGOTADO! El robot no logra llegar al punto. Saltando...")
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.proximo_punto()

    def map_callback(self, msg):
        if self.map_msg is not None: return
        self.get_logger().info("2/4. ¡Mapa recibido!")
        self.map_msg = msg
        if not self.nav_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("Nav2 no responde.")
            return
        self.procesar_y_arrancar(msg)

    def es_punto_valido(self, x_world, y_world):
        """Verifica si el punto es espacio libre."""
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
            self.get_logger().error("No hay zonas libres.")
            return

        # Ajuste de límites con el nuevo margen de 0.55m
        self.x_min = (min_x_idx * res) + origin_x + self.margen_seguridad
        self.x_max = (max_x_idx * res) + origin_x - self.margen_seguridad
        self.y_min = (min_y_idx * res) + origin_y + self.margen_seguridad
        self.y_max = (max_y_idx * res) + origin_y - self.margen_seguridad

        self.get_logger().info(f"Área segura calculada: X[{self.x_min:.2f} a {self.x_max:.2f}]")
        self.calcular_rutas()
        
        if self.rutas:
            self.ir_al_siguiente_punto()

    def calcular_rutas(self):
        y_actual = self.y_min
        ir_derecha = True
        paso_x = 0.60 # Distancia entre puntos de la misma línea
        
        while y_actual <= self.y_max:
            linea_x = []
            curr_x = self.x_min
            while curr_x <= self.x_max:
                linea_x.append(curr_x)
                curr_x += paso_x
            
            if not ir_derecha: linea_x.reverse()
            
            for x in linea_x:
                yaw = 0.0 if ir_derecha else 3.14
                if self.es_punto_valido(x, y_actual):
                    self.rutas.append((x, y_actual, yaw))
            
            y_actual += self.ancho_corte
            ir_derecha = not ir_derecha
            
        self.get_logger().info(f"Rutas generadas: {len(self.rutas)} puntos.")

    def ir_al_siguiente_punto(self):
        if self.punto_actual >= len(self.rutas):
            self.get_logger().info("¡MISIÓN DE COBERTURA COMPLETADA!")
            self.inicio_tiempo_punto = None
            return
            
        x, y, yaw = self.rutas[self.punto_actual]
        
        # --- NUEVO: Control de estado para evitar cascada ---
        self.esperando_meta = True 
        self.inicio_tiempo_punto = self.get_clock().now()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)
        
        self.get_logger().info(f"Punto {self.punto_actual + 1}/{len(self.rutas)} -> ({x:.2f}, {y:.2f})")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def revisar_progreso(self):
        if self.inicio_tiempo_punto is None or not self.esperando_meta:
            return
            
        tiempo_transcurrido = self.get_clock().now() - self.inicio_tiempo_punto
        if tiempo_transcurrido > Duration(seconds=45): # Bajamos a 45s para que sea más dinámico
            self.get_logger().warn("¡TIMEOUT! Saltando punto por tiempo...")
            
            # Cambiamos el estado ANTES de saltar para romper la cascada
            self.esperando_meta = False 
            
            if self.goal_handle:
                self.nav_client.cancel_goal_async(self.goal_handle)
            
            self.proximo_punto()

    def get_result_callback(self, future):
        # Solo avanzamos si el timeout no lo hizo primero
        if self.esperando_meta:
            self.esperando_meta = False
            self.proximo_punto()

    def proximo_punto(self):
        # Limpiamos el handle de la meta anterior
        self.goal_handle = None
        self.punto_actual += 1
        # Pequeña pausa de 0.5s para que la terminal no se sature
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
