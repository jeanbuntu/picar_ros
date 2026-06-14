"""
tracking_viz.launch.py

Launches robot_state_publisher with the picar_tracking URDF so that
the PiCar-X model appears live in RViz2 anchored to the 'camera' TF frame
published by tf_bridge.

Run alongside tf_bridge:
    # Terminal 1
    ./scripts/launch_tf_bridge.sh

    # Terminal 2
    source install/setup.bash
    ros2 launch picar_description tracking_viz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('picar_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'picar_tracking.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),
    ])
