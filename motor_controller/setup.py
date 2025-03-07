from setuptools import find_packages, setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='rammani@pi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bot_direction_publisher = motor_controller.bot_direction_publisher:main',
            'bot_direction_subscriber = motor_controller.bot_direction_subscriber:main',
            'single_motor_testing= motor_controller.single_motor_testing:main' 

        ],
    },
)
