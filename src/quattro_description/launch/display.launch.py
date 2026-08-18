from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('quattro_description')
    use_gui = LaunchConfiguration('use_gui')
    xacro_file = PathJoinSubstitution([package_share, 'urdf', 'quattro.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([package_share, 'rviz', 'quattro.rviz'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='false',
            description='Use joint_state_publisher_gui for URDF-only testing. '
                        'Normal Quattro operation receives /joint_states from quattro_serial_bridge.'
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui', output='screen', condition=IfCondition(use_gui),
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
