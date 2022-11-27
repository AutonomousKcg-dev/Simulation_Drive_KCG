
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
from ament_index_python import get_package_share_directory
import launch

import os

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)



# def get_package_share_directory(package_name):
#     """Return the absolute path to the share directory of the given package."""
#     return os.path.join(path_file
#         Path(FindPackage(package_name).perform(context)), "share", package_name
#     )


def generate_launch_description():
    """Launch controller_testing_node and mpc_controller."""
    minimal_simulator_pkg_prefix = get_package_share_directory("minimal_simulator")

    rviz_cfg_path = os.path.join(minimal_simulator_pkg_prefix, 'config/minimal_simulator.rviz')

    urdf_pkg_prefix = get_package_share_directory('il15_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/il15.urdf')

    sync_mode_param = DeclareLaunchArgument(
        'auto_sync_mode',
        default_value='False',
        description='simulator sync mode'
    )

    # Nodes
    sync =  Node(
        package="simulation_sync",
        node_executable="simulation_sync_main.py",
        node_name="simulation_sync_node",
        output="screen",        
        remappings=[
            ("vehicle_command", "/raw_command"),
        ],
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('auto_sync_mode'))
    )

    minimal_simulator = Node(
        package="minimal_simulator",
        node_executable="minimal_simulator_main.py",
        node_namespace="simulator/ego_vehicle",
        name="minimal_simulator_node",
        output="screen",        
        # LaunchConfiguration("minimal_simulator_param_file")
        parameters=[os.path.join(minimal_simulator_pkg_prefix, "param/vehicles/ego_vehicle.param.yaml")],
        remappings=[
            ("vehicle_state", "/vehicle_kinematic_state"),
            #("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/raw_command"),
            #("control_diagnostic", "/control/control_diagnostic"),
        ],
        on_exit=Shutdown()
    )


    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        #node_namespace='vehicle2',
        arguments=[str(urdf_path)]
        #arguments=[str(lexus_urdf_path)]
        
    )

    return LaunchDescription(
        [
            sync_mode_param,
            sync,
            # real_time_sim_param,
            rviz2,
            urdf_publisher,
            minimal_simulator,
        ]
    )
