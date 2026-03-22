from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'proyecto_modula'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # --- ESTA LÍNEA REGISTRA TUS LAUNCH FILES ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hfmd2',
    maintainer_email='hfmd2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'master_controller = proyecto_modula.master_controller:main',
       		'data_logger = proyecto_modula.data_logger:main',
        	'navigation_node = proyecto_modula.navigation_node:main',
        ],
    },
)
