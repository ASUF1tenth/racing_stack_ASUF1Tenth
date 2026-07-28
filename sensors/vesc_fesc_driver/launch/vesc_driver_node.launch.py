# Copyright 2020 F1TENTH Foundation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#   * Neither the name of the {copyright_holder} nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    port_val = LaunchConfiguration("port").perform(context)
    config_val = LaunchConfiguration("config").perform(context)
    node_name_val = LaunchConfiguration("node_name").perform(context)
    namespace_val = LaunchConfiguration("namespace").perform(context)

    params = [config_val]
    if port_val:
        params.append({"port": port_val})

    node = Node(
        package="vesc_fesc_driver",
        executable="vesc_driver_node",
        name=node_name_val,
        namespace=namespace_val,
        parameters=params,
        output="screen"
    )
    return [node]


def generate_launch_description():

    vesc_config = os.path.join(
        get_package_share_directory('vesc_fesc_driver'),
        'params',
        'vesc_config.yaml'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
            ),
        DeclareLaunchArgument(
            name="port",
            default_value="",
            description="VESC serial port override. If empty, uses the port from the config file.",
            ),
        DeclareLaunchArgument(
            name="node_name",
            default_value="vesc_driver_node",
            description="Name of the VESC/FESC driver node.",
            ),
        DeclareLaunchArgument(
            name="namespace",
            default_value="",
            description="Namespace of the node.",
            ),
        OpaqueFunction(function=launch_setup)
    ])
