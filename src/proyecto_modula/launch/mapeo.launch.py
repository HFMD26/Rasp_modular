import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Definimos las rutas primero
    pkg_share = get_package_share_directory('proyecto_modula')
    slam_config_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    ldlidar_launch_dir = os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Definimos el nodo de SLAM por separado antes del return
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        
        # 1. LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ldlidar_launch_dir, 'ld19.launch.py'))
        ),
        
        # 2. Master Controller
        Node(
            package='proyecto_modula',
            executable='master_controller',
            name='master_controller_node',
            output='screen'
        ),

        # 3. TF Estática
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.0', '0.0', '0.46', '0.0', '0.0', '0.0', 'base_link', 'base_laser', '--publish-period', '50'],
            output='screen'
        ),

        # 4. SLAM Toolbox (La variable que definimos arriba)
        slam_toolbox,

        # 5. Navigation 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # 6. RViz2
        #Node(
           # package='rviz2',
           # executable='rviz2',
            #name='rviz2',
            #output='screen'
        #)
    ])
