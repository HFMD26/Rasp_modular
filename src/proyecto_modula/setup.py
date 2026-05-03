from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'proyecto_modula'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Asegúrate de que cada línea termine en COMA ,
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu@correo.com',
    description='Descripción del paquete',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_controller = proyecto_modula.master_controller:main',
            'movimiento_aspirador = proyecto_modula.movimiento_aspirador:main',
        ],
    },
) # <--- ESTE PARÉNTESIS ES VITAL
