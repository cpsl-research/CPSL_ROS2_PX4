"""Console launch: px4_control_node plus its flight-controller transport, namespaced.

    ros2 launch px4_controller console.launch.py                      # backend:=xrce (default)
    ros2 launch px4_controller console.launch.py backend:=mavros      # MAVLink over USB via MAVROS
    ros2 launch px4_controller console.launch.py --show-args

backend:=xrce starts the Micro XRCE-DDS Agent (unless agent:=false) and the
node talking to /fmu/...; backend:=mavros starts mavros_node (mavros.launch.py,
config/px4_mavros.yaml) at <namespace>/mavros and the node talking to it. Both
implement the same console interface (README "Console interface").

Every altitude and timing limit is a launch argument passed to the node as a
parameter, so outdoor use only changes arguments. tf is remapped to the
namespace-relative `tf` / `tf_static` (docs/TF_AND_TOPICS.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_FCU_URL = '/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00:115200'


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='cpsl_uav_1',
                              description='ROS namespace for the node and tf'),
        DeclareLaunchArgument('backend', default_value='xrce', choices=['xrce', 'mavros'],
                              description='flight-controller transport: xrce (uXRCE-DDS, Ethernet) '
                                          'or mavros (MAVLink, e.g. USB)'),
        DeclareLaunchArgument('fcu_url', default_value=DEFAULT_FCU_URL,
                              description='backend:=mavros only: MAVROS fcu_url, '
                                          '/dev/serial/by-id/<id>:<baud> (baud ignored on USB)'),
        DeclareLaunchArgument('agent', default_value='true',
                              description='backend:=xrce only: start MicroXRCEAgent (false if one already runs)'),
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
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration('agent'), EqualsSubstitution(LaunchConfiguration('backend'), 'xrce'))),
    )

    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('px4_controller'), 'launch', 'mavros.launch.py')),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('backend'), 'mavros')),
    )

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        mavros,
        Node(
            package='px4_controller',
            executable='px4_control_node',
            name='px4_control_node',
            output='screen',
            parameters=[{
                'backend': LaunchConfiguration('backend'),
                'takeoff_altitude_m': _float('takeoff_altitude_m'),
                'min_altitude_m': _float('min_altitude_m'),
                'max_altitude_m': _float('max_altitude_m'),
                'max_climb_rate_mps': _float('max_climb_rate_mps'),
                'cmd_timeout_s': _float('cmd_timeout_s'),
            }],
        ),
    ])

    return LaunchDescription(args + [agent, group])
