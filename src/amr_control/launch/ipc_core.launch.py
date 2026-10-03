"""Start the non-driving AMR core stack on the robot IPC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    camera_enabled = LaunchConfiguration('camera_enabled')
    camera_display = LaunchConfiguration('camera_display')
    ekf_config = PathJoinSubstitution(
        [FindPackageShare('amr_control'), 'config', 'custom_ekf.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=EnvironmentVariable(
                'ROBOT_SERIAL_PORT',
                default_value='/dev/ttyUSB0',
            ),
            description='Motor/sensor controller serial device',
        ),
        DeclareLaunchArgument(
            'camera_enabled',
            default_value='true',
            description='Start the AprilTag camera localization node',
        ),
        DeclareLaunchArgument(
            'camera_display',
            default_value='false',
            description='Open the OpenCV monitor window (requires a display)',
        ),
        Node(
            package='amr_control',
            executable='robot_serial_bridge',
            name='robot_serial_bridge',
            output='screen',
            parameters=[{'port': serial_port}],
        ),
        Node(
            package='amr_control',
            executable='state_bridge',
            name='state_bridge_node',
            output='screen',
        ),
        Node(
            package='amr_control',
            executable='custom_ekf_node',
            name='custom_ekf_node',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='amr_control',
            executable='camera_node',
            name='pose_estimation_publisher',
            output='screen',
            condition=IfCondition(camera_enabled),
            parameters=[{
                'display_enabled': ParameterValue(
                    camera_display,
                    value_type=bool,
                ),
            }],
        ),
    ])
