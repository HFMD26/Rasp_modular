import rclpy
from rclpy.node import Node
import serial
import threading
import math
import time
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

# --- Configuración de Conexión ---
SERIAL_PORT = '/dev/serial0' 
SERIAL_BAUD = 115200 

class MasterControllerNode(Node): 
    def __init__(self):
        super().__init__('master_controller_node')
        self.get_logger().info('Iniciando Master Controller Unificado...')
        
        # 1. Conexión Serial
        self.esp32_serial = None
        try:
            self.esp32_serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            time.sleep(2) # Esperar a que la ESP32 se reinicie tras conectar
            self.get_logger().info(f'Conectado a ESP32 en {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Error de conexión serial: {e}. Ejecutando en modo SIMULACIÓN (sin ESP32).')

        # 2. Publicadores y Broadcasters para SLAM/Navegación
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3. Suscriptores (Escuchan a ROS para enviar a la ESP32)
        # Aseguramos suscribirnos a cmd_vel (el estándar del teleop)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.suction_sub = self.create_subscription(Bool, '/suction', self.suction_callback, 10)
        self.brushes_sub = self.create_subscription(Bool, '/brushes', self.brushes_callback, 10)
        self.aspersion_sub = self.create_subscription(Bool, '/aspersion', self.aspersion_callback, 10)
        
        # 4. Hilo de Lectura (Escucha a la ESP32 para enviar a ROS)
        if self.esp32_serial:
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()

    # ------------------------------------------------------------------
    # RECEPCIÓN DE DATOS (ESP32 -> RASPBERRY)
    # ------------------------------------------------------------------
    def read_serial_data(self):
        """Hilo que lee continuamente la odometría enviada por la ESP32"""
        while rclpy.ok() and self.esp32_serial:
            if self.esp32_serial.in_waiting > 0:
                try:
                    latest_line = ""
                    
                    # 1. DRENAR EL BUFFER: Leemos líneas rápidamente hasta ponernos al día
                    while self.esp32_serial.in_waiting > 0:
                        # errors='ignore' es CLAVE: evita que el nodo colapse si un byte llega mocho
                        latest_line = self.esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 2. Procesamos SOLAMENTE la última línea que leímos (la más reciente)
                    if latest_line.startswith("ODOM"):
                        parts = latest_line.split(',')
                        if len(parts) >= 6:
                            odom_x = float(parts[1])
                            odom_y = float(parts[2])
                            odom_yaw = float(parts[3])
                            odom_v = float(parts[4])
                            odom_w = float(parts[5])
                            
                            self.publish_odometry(odom_x, odom_y, odom_yaw, odom_v, odom_w)
                            
                except ValueError as e:
                    # A veces la conversión a float falla si llega una letra rara, lo ignoramos y seguimos
                    pass
                except Exception as e:
                    self.get_logger().warn(f"Error inesperado en serial: {e}")
            else:
                # 3. Pequeño respiro de 5ms para no fundir el CPU de la Raspberry
                time.sleep(0.005)
    def publish_odometry(self, x, y, yaw, v, w):
        """Publica el mensaje /odom y la transformada TF para SLAM"""
        now = self.get_clock().now().to_msg()
        
        # Crear cuaternión desde ángulo Yaw
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)

        # Enviar TF dinámica (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publicar mensaje Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w 
        self.odom_pub.publish(odom)
   

    # ------------------------------------------------------------------
    # ENVÍO DE COMANDOS (RASPBERRY -> ESP32)
    # ------------------------------------------------------------------
    def send_command_to_slave(self, command_str):
        """Agrega el salto de línea y envía el comando por Serial"""
        if self.esp32_serial:
            try:
                command_with_newline = command_str + "\n"
                self.esp32_serial.write(command_with_newline.encode('utf-8'))
                # self.get_logger().info(f'Enviado a ESP32: "{command_str}"') # Descomenta para debuggear
            except Exception as e:
                self.get_logger().error(f'Error al enviar comando: {e}')
        else:
             self.get_logger().debug(f'Modo SIMULACIÓN. Comando no enviado: "{command_str}"')

    def cmd_vel_callback(self, msg: Twist):
        """Convierte mensajes de movimiento en formato V,v,w"""
        v = msg.linear.x
        w = msg.angular.z
        
        # CORRECCIÓN DE ENVÍO: Mandamos V,velocidad_lineal,velocidad_angular
        # Ejemplo: V,0.39,1.41
        cmd = f"{v:.3f},{w:.3f}"
        
        # Log para verificar que el Teleop sí está entrando aquí
        self.get_logger().info(f'Recibido Teleop -> Enviando a ESP32: {cmd}')
        
        self.send_command_to_slave(cmd)

    def suction_callback(self, msg: Bool):
        state = 1 if msg.data else 0
        self.send_command_to_slave(f"S,{state}")

    def brushes_callback(self, msg: Bool):
        state = 1 if msg.data else 0
        self.send_command_to_slave(f"C,{state}")

    def aspersion_callback(self, msg: Bool):
        state = 1 if msg.data else 0
        self.send_command_to_slave(f"A,{state}")

# --- Ejecución ---
def main(args=None):
    rclpy.init(args=args)
    node = MasterControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.esp32_serial:
            node.esp32_serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
