from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


#locating other packages
pkg_px4_controller = get_package_share_directory('px4_controller')

#ROS2 launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_px4_controller, 'config', 'px4_joy.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    ),
    DeclareLaunchArgument(
        'joy_enable',
        default_value='false',
        description='Enable joystick if True',
    ),
    DeclareLaunchArgument(
        'control_enable',
        default_value='false',
        description='Enable control if True',
    ),
                          
]

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    joy_enable = LaunchConfiguration('joy_enable')
    control_enable = LaunchConfiguration('control_enable')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            convert_types=True,
            param_rewrites={}
        ),
        allow_substs=True,
    )

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        # SetRtmux aemap('/tf', namespace_str + '/tf'),
        # SetRemap('/tf_static', namespace_str + '/tf_static'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[configured_params],
            condition=IfCondition(joy_enable),
        ),
        Node(
            package='px4_controller',
            executable='px4_control_node',
            name='px4_control_node',
            condition=IfCondition(control_enable),
        ),
        Node(
            package='px4_controller',
            executable='px4_joy_node',
            name='px4_joy_node',
            condition=IfCondition(joy_enable),
        )

    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
