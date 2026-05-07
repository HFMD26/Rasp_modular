import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Rutas de los paquetes
    pkg_modula = get_package_share_directory('proyecto_modula')
    ldlidar_dir = get_package_share_directory('ldlidar_stl_ros2')
    
    # Ruta del mapa que guardaste anteriormente
    map_file = os.path.join(pkg_modula, 'maps', 'mapa_proyecto.yaml')

    return LaunchDescription([
        
        # 2. Driver del LiDAR (LD19)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ldlidar_dir, 'launch', 'ld19.launch.py')
            )
        ),
        
        # 3. Servidor del Mapa (Necesario para que exista el frame 'map')
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}, {'use_sim_time': False}]
        ),

        # 4. Lifecycle Manager (Para activar el servidor de mapas automáticamente)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True}, {'node_names': ['map_server']}]
        ),
        
        # 5. Tu script de Línea Recta
        # Asegúrate de haberlo registrado como 'linea_recta' en entry_points de setup.py
        Node(
            package='proyecto_modula',
            executable='linea_recta',
            name='test_linea_recta',
            output='screen'
        ),

        # 6. RViz2 para ver qué está pasando
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
