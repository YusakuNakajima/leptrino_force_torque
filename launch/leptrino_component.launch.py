from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch arguments
    comport = LaunchConfiguration('comport')
    sampling_rate = LaunchConfiguration('sampling_rate')
    frame_id = LaunchConfiguration('frame_id')
    container_name = LaunchConfiguration('container_name')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'comport',
            default_value='/dev/ttyACM0',
            description='Path to the COM port'
        ),
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='1200.0',
            description='Publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='leptrino_link',
            description='Frame ID for the sensor'
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='leptrino_container',
            description='Name of the component container'
        ),

        # Component container
        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='leptrino_force_torque',
                    plugin='LeptrinoNode',
                    name='leptrino_force_torque_node',
                    parameters=[
                        {
                            'com_port': comport,
                            'frame_id': frame_id,
                            'rate': sampling_rate
                        }
                    ]
                )
            ],
            output='screen'
        )
    ])