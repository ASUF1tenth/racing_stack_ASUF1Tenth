from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_type', default_value='single_track_model'),
        DeclareLaunchArgument('racecar_version'),
        DeclareLaunchArgument('floor', default_value='dubi_hoons'),

        DeclareLaunchArgument('odom_vesc_topic', default_value='/vesc/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('odom_topic', default_value='/state_estimation/odom'),

        # Covariance matrices. Kept as plain literals (not launch args) since ROS2
        # launch arguments are always strings and can't cleanly carry float arrays.
        # Q needs to match the state-dimension of the selected model.
        Node(
            package='state_estimation',
            executable='ukf_node',
            name='ukf_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'frequency': 100,
                'racecar_version': LaunchConfiguration('racecar_version'),
                'model_type': LaunchConfiguration('model_type'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'vesc_topic': LaunchConfiguration('odom_vesc_topic'),
                'floor': LaunchConfiguration('floor'),
                'R_imu': [0.1, 0.1, 1000000.0, 1000000.0],
                'R_vesc': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'Q': [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
            }],
        ),
    ])
