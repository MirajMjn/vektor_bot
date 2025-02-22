from setuptools import setup
import os
from glob import glob

package_name = 'vektor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs all files ending in .launch.py from the launch directory
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools', 'my_interfaces'],
    zip_safe=True,
    maintainer='Miraj Maharjan',
    maintainer_email='miraj.077bei022@acem.edu.np',
    description='VE-KT-O-R :  VEctor KinemaTics Omnidirectional Robot - A omnidirectional drive robot that uses three omni wheels for holonomic omnidirectional drive, that we are making for our B.E. final year project.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'node_executable = vektor.node:main',
            'teleop_node = vektor.teleop_node:main',
            'kinematics_node = vektor.kinematics_node:main',
            'PID_control_node = vektor.PID_control_node:main',
            'motor_interface_node = vektor.motor_interface_node:main',
            'serial_broadcaster_node = vektor.serial_broadcaster_node:main',
        ],
    },
)
