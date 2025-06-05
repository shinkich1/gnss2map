# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():
    # パラメータファイルの設定
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('gnss2map'),
                'config', 'params', 'default.param.yaml')),
        ],
        description='gnss2map param file path'
    )

    # GaussKrugerノード
    gausskruger_node = Node(
        package="gnss2map",
        executable="gauss_kruger_node",
        name="gauss_kruger_node",
        parameters=[params_file],
        output="screen"
    )

    # GnssPoserノード
    gnssposer_node = Node(
        package="gnss2map",
        executable="gnss_poser_node",
        name="gnss_poser_node",
        parameters=[params_file],
        output="screen"
    )

    # LaunchDescriptionに追加
    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(gausskruger_node)
    ld.add_action(gnssposer_node)

    return ld