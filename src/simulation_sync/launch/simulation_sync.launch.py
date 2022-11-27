
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os


def generate_launch_description():


    # Arguments

    

    # Nodes

    simulation_sync = Node(
        package="simulation_sync",
        node_executable="simulation_sync_main.py",
        # node_namespace="vrx",
        node_name="simulation_sync_node",
        output="screen",
        remappings=[
            ("vehicle_command", "/raw_command"),
        ],
        on_exit=Shutdown()
    )


    return LaunchDescription(
        [
            simulation_sync
        ]
    )
