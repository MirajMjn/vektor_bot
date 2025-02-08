from glob import glob
from setuptools import find_packages, setup

package_name = 'vektor'

setup(
    name='vektor',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs all files ending in .launch.py from the launch directory
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miraj Maharjan',
    maintainer_email='miraj.077bei022@acem.edu.np',
    description='VE-KT-O-R :  VEctor KinemaTics Omnidirectional Robot - A omnidirectional drive robot that uses three omni wheels for holonomic omnidirectional drive, that we are making for our B.E. final year project.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'node_executable = vektor.node:main',
        ],
    }
)
