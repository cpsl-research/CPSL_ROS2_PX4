import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='px4_controller',
            executable='px4_keyop_node',
            namespace="CPSL_UAV_1",
            name='px4_keyop_node'),
        launch_ros.actions.Node(
            package='px4_controller',
            executable='px4_control_node',
            namespace="CPSL_UAV_1",
            name='px4_control_node')
  ])