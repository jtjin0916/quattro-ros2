import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_quattro = get_package_share_directory('quattro')
    params = os.path.join(pkg_quattro, 'config', 'gim_params.yaml')
    joy_params = os.path.join(pkg_quattro, 'config', 'joy_params.yaml')
    calib = os.path.join(pkg_quattro, 'config', 'quattro_servo_calib.yaml')

    return LaunchDescription([
        Node(
            package='joy', executable='joy_node', name='quattro_joy',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        ),
        Node(
            package='quattro', executable='teleop_node', name='quattro_teleop', output='screen',
            parameters=[joy_params],
        ),
        Node(
            package='quattro', executable='quattro_sm', name='quattro_sm', output='screen',
            parameters=[{'frequency': 100.0}],
        ),
        Node(
            package='quattro', executable='quattro_commander', name='quattro_commander', output='screen',
            parameters=[params, joy_params, calib, {
                'control_rate_hz': 100.0,
                'enable_direct_twist_input': False,
                'actuator_kp': 60.0,
                'actuator_kd': 0.8,
                'torque_ff_default': 0.0,
            }],
        ),
        Node(
            package='quattro', executable='quattro_serial_bridge', name='quattro_serial_bridge', output='screen',
            parameters=[{'port': '/dev/ttyACM0', 'baudrate': 921600}],
        ),
        Node(
            package='quattro', executable='bno085_node', name='bno085_node', output='screen',
        ),
    ])
