import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Buscamos la ruta exacta donde está instalado el paquete del LiDAR
    ldlidar_launch_dir = os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch')

    return LaunchDescription([
        
        # 1. Mandamos a llamar al launch oficial del LD19
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ldlidar_launch_dir, 'ld19.launch.py')
            )
        ),
        
        # 2. Tu Master Controller
        Node(
            package='proyecto_modula',
            executable='master_controller',
            name='master_controller_node',
            output='screen'
        ),

        # 3. RViz2 para visualización
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
