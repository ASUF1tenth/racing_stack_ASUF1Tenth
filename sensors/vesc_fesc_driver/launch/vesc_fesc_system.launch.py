import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vesc_driver_dir = get_package_share_directory('vesc_fesc_driver')

    # Launch configurations
    fesc_port = LaunchConfiguration('fesc_port')
    vesc_port = LaunchConfiguration('vesc_port')
    config = LaunchConfiguration('config')

    speed_to_erpm_gain = LaunchConfiguration('speed_to_erpm_gain')
    speed_to_erpm_offset = LaunchConfiguration('speed_to_erpm_offset')
    steering_angle_to_servo_gain = LaunchConfiguration('steering_angle_to_servo_gain')
    steering_angle_to_servo_offset = LaunchConfiguration('steering_angle_to_servo_offset')
    wheelbase = LaunchConfiguration('wheelbase')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_tf = LaunchConfiguration('publish_tf')

    # Declare arguments
    declare_fesc_port_arg = DeclareLaunchArgument(
        'fesc_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the FESC (BLDC motor control and feedback)'
    )

    declare_vesc_port_arg = DeclareLaunchArgument(
        'vesc_port',
        default_value='/dev/ttyACM1',
        description='Serial port for the VESC (Servo control and IMU feedback)'
    )

    vesc_config_default = os.path.join(vesc_driver_dir, 'params', 'vesc_config.yaml')
    declare_config_arg = DeclareLaunchArgument(
        'config',
        default_value=vesc_config_default,
        description='Path to VESC configuration YAML file'
    )

    # Calibration & system parameters arguments
    declare_speed_to_erpm_gain_arg = DeclareLaunchArgument(
        'speed_to_erpm_gain',
        default_value='4614.0',
        description='Gain parameter to convert m/s to ERPM'
    )

    declare_speed_to_erpm_offset_arg = DeclareLaunchArgument(
        'speed_to_erpm_offset',
        default_value='0.0',
        description='Offset parameter to convert m/s to ERPM'
    )

    declare_steering_angle_to_servo_gain_arg = DeclareLaunchArgument(
        'steering_angle_to_servo_gain',
        default_value='-0.66845076099',
        description='Gain parameter to convert steering angle (rad) to servo command'
    )

    declare_steering_angle_to_servo_offset_arg = DeclareLaunchArgument(
        'steering_angle_to_servo_offset',
        default_value='0.5',
        description='Offset parameter to convert steering angle (rad) to servo command'
    )

    declare_wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.2',
        description='Wheelbase of the vehicle (meters)'
    )

    declare_odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame ID'
    )

    declare_publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Whether to publish transform from odom_frame to base_frame'
    )

    # 1. FESC Driver Node (handling motor commands and speed feedback)
    fesc_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_driver_dir, 'launch', 'vesc_driver_node.launch.py')
        ),
        launch_arguments={
            'config': config,
            'port': fesc_port,
            'namespace': 'fesc',
            'node_name': 'fesc_driver_node'
        }.items()
    )

    # 2. VESC Driver Node (handling servo commands and IMU feedback)
    vesc_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_driver_dir, 'launch', 'vesc_driver_node.launch.py')
        ),
        launch_arguments={
            'config': config,
            'port': vesc_port,
            'namespace': 'vesc',
            'node_name': 'vesc_driver_node'
        }.items()
    )

    # 3. VESC-FESC Topic Bridge Node
    vesc_fesc_bridge_node = Node(
        package='vesc_fesc_driver',
        executable='vesc_fesc_bridge.py',
        name='vesc_fesc_bridge',
        output='screen'
    )

    # 4. Ackermann to VESC Node (from vesc_ackermann package)
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[
            {
                'speed_to_erpm_gain': speed_to_erpm_gain,
                'speed_to_erpm_offset': speed_to_erpm_offset,
                'steering_angle_to_servo_gain': steering_angle_to_servo_gain,
                'steering_angle_to_servo_offset': steering_angle_to_servo_offset,
            }
        ]
    )

    # 5. VESC to Odom Node (from vesc_ackermann package)
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[
            {
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'speed_to_erpm_gain': speed_to_erpm_gain,
                'speed_to_erpm_offset': speed_to_erpm_offset,
                'use_servo_cmd_to_calc_angular_velocity': True,
                'steering_angle_to_servo_gain': steering_angle_to_servo_gain,
                'steering_angle_to_servo_offset': steering_angle_to_servo_offset,
                'wheelbase': wheelbase,
                'publish_tf': publish_tf,
            }
        ]
    )

    return LaunchDescription([
        declare_fesc_port_arg,
        declare_vesc_port_arg,
        declare_config_arg,
        declare_speed_to_erpm_gain_arg,
        declare_speed_to_erpm_offset_arg,
        declare_steering_angle_to_servo_gain_arg,
        declare_steering_angle_to_servo_offset_arg,
        declare_wheelbase_arg,
        declare_odom_frame_arg,
        declare_base_frame_arg,
        declare_publish_tf_arg,
        fesc_driver_node,
        vesc_driver_node,
        vesc_fesc_bridge_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node
    ])
