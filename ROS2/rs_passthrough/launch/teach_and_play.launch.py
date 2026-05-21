from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    master_port_arg = DeclareLaunchArgument(
        'master_port', default_value=TextSubstitution(text='/dev/ttyUSB0'),
        description='Serial port for the master arm'
    )
    alpha_port_arg = DeclareLaunchArgument(
        'alpha_port', default_value=TextSubstitution(text='/dev/ttyUSB1'),
        description='Serial port for the Alpha 5 manipulator'
    )
    return LaunchDescription([
        master_port_arg,
        alpha_port_arg,
        Node(
            package='rs_passthrough',
            executable='serial_passthrough',
            namespace='master',
            parameters=[{'serial_port': LaunchConfiguration('master_port')}],
        ),
        Node(
            package='rs_passthrough',
            executable='serial_passthrough',
            namespace='alpha',
            parameters=[{
                'serial_port': LaunchConfiguration('alpha_port'),
                'half_duplex': True,
            }],
        ),
        Node(
            package='rs_passthrough',
            executable='joint_telemetry',
            namespace='alpha',
        )
    ])
