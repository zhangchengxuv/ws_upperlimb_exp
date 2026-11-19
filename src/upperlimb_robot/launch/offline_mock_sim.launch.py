from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='100',
        description='模拟控制频率（Hz），默认 100'
    )

    mock_node = Node(
        package='upperlimb_robot',
        executable='mock_robot_node.py',
        name='mock_robot_node',
        output='screen',
        parameters=[{
            'control_hz': ParameterValue(LaunchConfiguration('control_hz'), value_type=int),
        }],
    )

    gui_node = Node(
        package='ros_gui',
        executable='ros_gui_node',
        name='ros_gui_node',
        output='screen',
    )

    return LaunchDescription([
        control_hz_arg,
        mock_node,
        gui_node,
    ])
