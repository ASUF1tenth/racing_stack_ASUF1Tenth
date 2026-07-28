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

    return LaunchDescription([
        declare_fesc_port_arg,
        declare_vesc_port_arg,
        declare_config_arg,
        fesc_driver_node,
        vesc_driver_node,
        vesc_fesc_bridge_node
    ])
