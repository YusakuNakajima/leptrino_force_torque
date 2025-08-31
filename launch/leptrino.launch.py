from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    comport = LaunchConfiguration('comport')
    sampling_rate = LaunchConfiguration('sampling_rate')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'comport',
            default_value='/dev/ttyACM0',
            description='Path to the COM port'),
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='1200.0',
            description='Publishin  g rate in Hz'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='leptrino_frame',
            description='Frame ID for the sensor'),

        Node(
            package='leptrino_force_torque',
            executable='leptrino_force_torque',
            name='leptrino',
            output='screen',
            parameters=[
                {
                    'com_port': comport,
                    'frame_id': frame_id,
                    'rate': LaunchConfiguration('sampling_rate', default=100.0)
                }
            ]
        )
    ])