"""Console launch: Micro XRCE-DDS Agent plus px4_control_node, namespaced.

    ros2 launch px4_controller console.launch.py
    ros2 launch px4_controller console.launch.py --show-args

Every altitude and timing limit is a launch argument passed to the node as a
parameter, so outdoor use only changes arguments. tf is remapped to the
namespace-relative `tf` / `tf_static` (docs/TF_AND_TOPICS.md).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='cpsl_uav_1',
                              description='ROS namespace for the node and tf'),
        DeclareLaunchArgument('agent', default_value='true',
                              description='start MicroXRCEAgent (false if one already runs)'),
        DeclareLaunchArgument('agent_port', default_value='8888',
                              description='UDP port of the Micro XRCE-DDS Agent'),
        DeclareLaunchArgument('takeoff_altitude_m', default_value='0.75',
                              description='altitude held after takeoff, metres'),
        DeclareLaunchArgument('min_altitude_m', default_value='0.5',
                              description='lowest commanded altitude while flying, metres'),
        DeclareLaunchArgument('max_altitude_m', default_value='2.5',
                              description='highest commanded altitude, metres'),
        DeclareLaunchArgument('max_climb_rate_mps', default_value='0.3',
                              description='clamp on linear.z integrated into the altitude target, m/s'),
        DeclareLaunchArgument('cmd_timeout_s', default_value='0.5',
                              description='hover when cmd_vel/deadman is older than this, seconds'),
    ]

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('agent_port')],
        name='microxrce_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('agent')),
    )

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='px4_controller',
            executable='px4_control_node',
            name='px4_control_node',
            output='screen',
            parameters=[{
                'takeoff_altitude_m': _float('takeoff_altitude_m'),
                'min_altitude_m': _float('min_altitude_m'),
                'max_altitude_m': _float('max_altitude_m'),
                'max_climb_rate_mps': _float('max_climb_rate_mps'),
                'cmd_timeout_s': _float('cmd_timeout_s'),
            }],
        ),
    ])

    return LaunchDescription(args + [agent, group])
