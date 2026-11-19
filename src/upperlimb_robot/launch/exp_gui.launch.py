from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gui_node = Node(
        package='ros_gui',
        executable='ros_gui_node',
        name='ros_gui_node',
        output='screen',
    )
    return LaunchDescription([gui_node])
