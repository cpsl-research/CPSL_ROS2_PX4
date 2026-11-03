"""MAVROS for the px4_controller MAVROS backend (MAVLink, e.g. the Pixhawk's USB-C).

    ros2 launch px4_controller mavros.launch.py
    ros2 launch px4_controller mavros.launch.py fcu_url:=/dev/serial/by-id/<id>:115200

Included by console.launch.py when backend:=mavros, inside its namespace.
mavros_node gets no namespace of its own: its UAS node is named `mavros`, so
its topics land under /<namespace>/mavros/... (a `mavros` namespace on top
would give /<namespace>/mavros/mavros/...). On its own it only starts
mavros_node: it reads the FCU and sends MAVROS's heartbeat, and commands
nothing (no setpoints, mode changes or arming come from MAVROS itself).

fcu_url: use the stable /dev/serial/by-id/ name, never /dev/ttyACM<N> (the
number changes when the FC re-enumerates). The port is USB CDC ACM, so the
baud is ignored by the device, but MAVROS's `path:baud` syntax needs one.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_FCU_URL = '/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00:115200'


def generate_launch_description():
    config = os.path.join(get_package_share_directory('px4_controller'), 'config', 'px4_mavros.yaml')
    args = [
        DeclareLaunchArgument('fcu_url', default_value=DEFAULT_FCU_URL,
                              description='MAVROS FCU url: /dev/serial/by-id/<id>:<baud> (baud ignored on USB)'),
        DeclareLaunchArgument('gcs_url', default_value='',
                              description='optional GCS bridge, e.g. udp://@<qgc-host> (keep QGC off the serial port)'),
        DeclareLaunchArgument('tgt_system', default_value='1', description='MAVLink system id of the FCU'),
        DeclareLaunchArgument('tgt_component', default_value='1', description='MAVLink component id of the FCU'),
    ]
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[config, {
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'fcu_protocol': 'v2.0',
        }],
        # MAVROS sends no tf (config), but keep any tf it might send off the
        # global /tf, as console.launch.py does for the node.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    return LaunchDescription(args + [mavros])
