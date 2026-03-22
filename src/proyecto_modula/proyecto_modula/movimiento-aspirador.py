#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
import math

def euler_a_quaternion(yaw):
    """Función manual para no depender de librerías externas en la Raspberry"""
    q = Quaternion()class CortadorSeguro(Node):
    def __init__(self):
        super().__init__('cerebro_cortador_final')
        
        self.map_msg = None
        self.margen_seguridad = 0.10
        self.ancho_corte = 0.30
        
        # Almacenaremos la lista de coordenadas validas aqui­
        self.rutas = []
        self.punto_actual = 0
        
        # 1. Cliente de accian para Nav2 (El reemplazo de move_base)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. Subscripcion al mapa (El reemplazo asi­ncrono de wait_for_message)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            1) # QoS de 1 para obtener solo el ultimo mapa
        
        self.get_logger().info("1/4. Esperando mapa en /map...")
            q.w = math.cos(yaw / 2.0)
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(yaw / 2.0)
        return q

class CortadorSeguro(Node):
    def __init__(self):
        super().__init__('cerebro_cortador_final')
        
        self.map_msg = None
        self.margen_seguridad = 0.10
        self.ancho_corte = 0.30
        
        # Almacenaremos la lista de coordenadas vÃ¡lidas aquÃ­
        self.rutas = []
        self.punto_actual = 0
        
        # 1. Cliente de acciÃ³n para Nav2 (El reemplazo de move_base)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. SubscripciÃ³n al mapa (El reemplazo asÃ­ncrono de wait_for_message)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            1) # QoS de 1 para obtener solo el Ãºltimo mapa
        
        self.get_logger().info("1/4. Esperando mapa en /map...")

    def map_callback(self, msg):
        self.get_logger().info("2/4. ¡Mapa recibido! Desconectando subscripción...")
        self.map_msg = msg
        
        # Destruimos la subscripción porque solo necesitamos el mapa una vez
        self.destroy_subscription(self.map_sub)
        
        # Esperar a que Nav2 esté en línea
        self.get_logger().info("3/4. Conectando con Nav2 (navigate_to_pose)...")
        self.nav_client.wait_for_server()
        
        # Iniciar la lógica
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

        # Verificar si está dentro del array
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False

        # Obtener valor del píxel (0=Libre, 100=Pared, -1=Desconocido)
        index = grid_y * width + grid_x
        valor_pixel = self.map_msg.data[index]

        return valor_pixel == 0

    def procesar_y_arrancar(self, msg):
        self.get_logger().info("Calculando límites del área...")
        
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
                y = i // width
                x = i % width
                if x < min_x_idx: min_x_idx = x
                if x > max_x_idx: max_x_idx = x
                if y < min_y_idx: min_y_idx = y
                if y > max_y_idx: max_y_idx = y

        if not encontro_libre:
            self.get_logger().error("El mapa está vacío.")
            return

        self.x_min = (min_x_idx * res) + origin_x + self.margen_seguridad
        self.x_max = (max_x_idx * res) + origin_x - self.margen_seguridad
        self.y_min = (min_y_idx * res) + origin_y + self.margen_seguridad
        self.y_max = (max_y_idx * res) + origin_y - self.margen_seguridad

        self.get_logger().info(f"Límites brutos: X[{self.x_min:.2f}, {self.x_max:.2f}]")
        self.calcular_rutas()

    def calcular_rutas(self):
        y_actual = self.y_min
        ir_derecha = True

        self.get_logger().info("4/4. CALCULANDO RUTAS VÁLIDAS...")

        # Primero, procesamos y guardamos todos los puntos válidos en una lista
        while y_actual <= self.y_max:
            if ir_derecha:
                puntos_linea = [(self.x_min, y_actual, 0.0), (self.x_max, y_actual, 0.0)]
            else:
                puntos_linea = [(self.x_max, y_actual, 3.14), (self.x_min, y_actual, 3.14)]def calcular_rutas(self):
        y_actual = self.y_min
        ir_derecha = True

        self.get_logger().info("4/4. CALCULANDO RUTAS VÃ�LIDAS...")

        # Primero, procesamos y guardamos todos los puntos vÃ¡lidos en una lista
        while y_actual <= self.y_max:
            if ir_derecha:
                puntos_linea = [(self.x_min, y_actual, 0.0), (self.x_max, y_actual, 0.0)]
            else:
                puntos_linea = [(self.x_max, y_actual, 3.14), (self.x_min, y_actual, 3.14)]

	for (x, y, yaw) in puntos_linea:
                if self.es_punto_valido(x, y):
                    self.rutas.append((x, y, yaw))
                else:
                    self.get_logger().warn(f"  Descartando X={x:.2f}, Y={y:.2f} (Pared o Vacío)")

            y_actual += self.ancho_corte
            ir_derecha = not ir_derecha

        if not self.rutas:
            self.get_logger().error("No se encontraron puntos válidos en el mapa.")
        else:
            self.get_logger().info(f" Se generaron {len(self.rutas)} objetivos. Iniciando movimiento...")
            self.rutas.append((0.0, 0.0, 0.0)) # Regresar al inicio al final
            self.ir_al_siguiente_punto() # Detonamos el primer movimiento

    def ir_al_siguiente_punto(self):
        """Envía el objetivo a Nav2 asíncronamente"""
        if self.punto_actual >= len(self.rutas):
            self.get_logger().info(" ¡Trabajo terminado! Robot regresó a la base.")
            return

        x, y, yaw = self.rutas[self.punto_actual]
        self.get_logger().info(f"-> Viajando al objetivo {self.punto_actual + 1}: X={x:.2f}, Y={y:.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = euler_a_quaternion(yaw)

        # Enviar meta y configurar callbacks para no bloquear el sistema
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Verifica si Nav2 aceptó nuestro destino"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rechazó el destino (Posiblemente inaccesible). Saltando al siguiente...")
            self.punto_actual += 1
            self.ir_al_siguiente_punto()
            return

        # Si aceptó, esperamos asíncronamente a que llegue
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Se ejecuta cuando el robot llega a su destino o falla en el camino"""
        # No importa si llegó bien o se atoró, pasamos al siguiente punto del zig-zag
        self.punto_actual += 1
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
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
