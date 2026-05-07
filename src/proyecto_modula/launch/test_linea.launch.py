import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('proyecto_modula') # Cambia por tu nombre de paquete
    map_file = os.path.join(pkg_dir, 'maps', 'mapa_proyecto.yaml')

    return LaunchDescription([
        # 1. Servidor del Mapa
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{'yaml_filename': map_file}, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True}, {'node_names': ['map_server']}]
        ),

        # 2. Tu script de movimiento
        Node(
            package='proyecto_modula',
            executable='linea_recta', # Asegúrate que el archivo tenga permisos de ejecución
            output='screen'
        )
    ])
