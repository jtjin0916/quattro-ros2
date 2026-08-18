from setuptools import find_packages, setup
from glob import glob

package_name = 'quattro'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/policies', glob('policies/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taejin Jo',
    maintainer_email='jtjin0916@gmail.com',
    description='Quattro quadruped ROS2 control and Teensy serial bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quattro_commander = quattro.quattro_commander:main',
            'quattro_sm = quattro.quattro_sm:main',
            'quattro_serial_bridge = quattro.quattro_serial_bridge:main',
            'teleop_node = quattro.teleop_node:main',
            'bno085_node = quattro.bno085_node:main',
            'keyboard_teleop = quattro.keyboard_teleop:main',
            'keyboard_teleop_gim = quattro.keyboard_teleop_gim:main',
            'motor_calibrator_ros2 = quattro.motor_calibrator_ros2:main',
        ],
    },
)
