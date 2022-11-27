
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
    minimal_simulator_pkg_prefix = get_package_share_directory("minimal_simulator")
    # minimal_simulator_param_file = os.path.join(
    #     minimal_simulator_pkg_prefix, "param/defaults.param.yaml"
    # )
    # minimal_simulator_il15_sim_param_file = os.path.join(path_file
    #     minimal_simulator_pkg_prefix, "param/il15_sim.param.yaml"
    # )
    # rviz_cfg_path = os.path.join(minimal_simulator_pkg_prefix, 'config/minimal_simulator.rviz')

    # Arguments
    # minimal_simulator_param = DeclareLaunchArgument(
    #     "minimal_simulator_param_file",
    #     default_value=rviz_cfg_path,#minimal_simulator_param_file,
    #     description="Path to config file for minimal simulator",
    # )
    # real_time_sim_param = DeclareLaunchArgument(
    #     'real_time_sim',
    #     default_value='False',
    #     description='Launch RVIZ2 in addition to other nodes'
    # )


    # urdf_pkg_prefix = get_package_share_directory('il15_description')
    # urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/il15.urdf')

    sync_mode_param = DeclareLaunchArgument(
        'auto_sync_mode',
        default_value='False',
        description='simulator sync mode'
    )

    sync =  Node(
        package="simulation_sync",
        executable="simulation_sync_main.py",
        name="simulation_sync_node",
        output="screen",        
        remappings=[
            ("vehicle_command", "/raw_command"),
        ],
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('auto_sync_mode'))
    )

    minimal_simulator = Node(
        package="minimal_simulator",
        executable="minimal_simulator_main.py",
        namespace="simulator/ego_vehicle",
        name="minimal_simulator_node",
        output="screen",        
        # LaunchConfiguration("minimal_simulator_param_file")
        parameters=[os.path.join(minimal_simulator_pkg_prefix, "param/vehicles/ego_vehicle_tf.param.yaml")],
        remappings=[
            ("vehicle_state", "/vehicle_kinematic_state"),
            #("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/raw_command"),
            #("control_diagnostic", "/control/control_diagnostic"),
        ],
        on_exit=Shutdown()
    )
   
    # rviz2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', str(rviz_cfg_path)],
    #     # arguments=['-d', str(minimal_simulator_param)],
    # )
    # urdf_publisher = Node(
    #     package='robot_state_publisher',
    #     node_executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     #node_namespace='vehicle2',
    #     arguments=[str(urdf_path)]
    #     #arguments=[str(lexus_urdf_path)]
    # )





    return LaunchDescription(
        [
            sync_mode_param,
            sync,
            # rviz2,
            # urdf_publisher,
            minimal_simulator,
        ]
    )
