from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='100',
        description='控制环频率（Hz），默认 100，可一键切回 50'
    )
    state_hz_arg = DeclareLaunchArgument(
        'state_hz',
        default_value=LaunchConfiguration('control_hz'),
        description='状态采集线程频率（Hz），默认与 control_hz 同步'
    )
    publish_divider_arg = DeclareLaunchArgument(
        'publish_divider',
        default_value='5',
        description='状态发布分频（100 Hz 下默认 5 -> 20 Hz 发布给 GUI）'
    )
    log_dir_arg = DeclareLaunchArgument(
        'log_output_dir',
        default_value='data',
        description='异步 CSV 日志输出目录'
    )
    ecat_cfg_arg = DeclareLaunchArgument(
        'ecat_config_path',
        default_value='',
        description='EtherCAT ENI 配置文件路径，留空自动使用包内 config/Gecat.eni'
    )

    control_node = Node(
        package='upperlimb_robot',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[{
            'control_hz': ParameterValue(LaunchConfiguration('control_hz'), value_type=int),
            'state_hz': ParameterValue(LaunchConfiguration('state_hz'), value_type=int),
            'publish_divider': ParameterValue(LaunchConfiguration('publish_divider'), value_type=int),
            'log_output_dir': LaunchConfiguration('log_output_dir'),
            'ecat_config_path': LaunchConfiguration('ecat_config_path'),
        }],
    )

    return LaunchDescription([
        control_hz_arg,
        state_hz_arg,
        publish_divider_arg,
        log_dir_arg,
        ecat_cfg_arg,
        control_node,
    ])
