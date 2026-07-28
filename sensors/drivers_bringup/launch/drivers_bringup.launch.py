import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vesc_driver_share = get_package_share_directory('vesc_fesc_driver')
    sllidar_ros2_share = get_package_share_directory('sllidar_ros2')

    # Launch Configurations
    fesc_port = LaunchConfiguration('fesc_port')
    vesc_port = LaunchConfiguration('vesc_port')
    vesc_config = LaunchConfiguration('vesc_config')

    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')

    # Declare Launch Arguments
    declare_fesc_port_arg = DeclareLaunchArgument(
        'fesc_port',
        default_value='/dev/ttyACM0',
        description='Serial port for FESC (BLDC motor control)'
    )

    declare_vesc_port_arg = DeclareLaunchArgument(
        'vesc_port',
        default_value='/dev/ttyACM1',
        description='Serial port for VESC (Servo control and IMU)'
    )

    default_vesc_config = os.path.join(vesc_driver_share, 'params', 'vesc_config.yaml')
    declare_vesc_config_arg = DeclareLaunchArgument(
        'vesc_config',
        default_value=default_vesc_config,
        description='Path to VESC configuration YAML file'
    )

    declare_channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Channel type for SLLiDAR'
    )

    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for SLLiDAR A1'
    )

    declare_serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate for SLLiDAR A1'
    )

    declare_frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for LiDAR scan data'
    )

    declare_inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Invert scan data'
    )

    declare_angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Enable angle compensation for LiDAR scan'
    )

    # 1. Include VESC / FESC drivers launch file
    vesc_fesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_driver_share, 'launch', 'vesc_fesc_drivers.launch.py')
        ),
        launch_arguments={
            'fesc_port': fesc_port,
            'vesc_port': vesc_port,
            'config': vesc_config,
        }.items()
    )

    # 2. Include SLLiDAR A1 driver launch file
    sllidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_share, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
        }.items()
    )

    return LaunchDescription([
        declare_fesc_port_arg,
        declare_vesc_port_arg,
        declare_vesc_config_arg,
        declare_channel_type_arg,
        declare_serial_port_arg,
        declare_serial_baudrate_arg,
        declare_frame_id_arg,
        declare_inverted_arg,
        declare_angle_compensate_arg,
        vesc_fesc_launch,
        sllidar_a1_launch,
    ])
