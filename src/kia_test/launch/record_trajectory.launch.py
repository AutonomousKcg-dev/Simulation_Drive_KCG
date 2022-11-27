# To-Do
# 1: make recording trajectory launch file within the lgsvl
# 2: see how you can recive gps input from lgsvl
# 3: record the trajcetory under an understandable name _lgsvl

from inspect import Arguments
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    # --------------------------------- Params -------------------------------
    traj_file_name = get_share_file(
        'kia_test', 'param/trajectories/trajectory_test_1.txt')

    record_traj = Node(
        package='record_traj',
        executable='record_traj_exe',
        # node_namespace='ego_vehicle',
        output='screen',
        parameters=[{"record_file": traj_file_name}
                    ],
        remappings=[
            ("/vehicle_kinematic_state", "vehicle_state")
        ]
    )

    # joystick_launch_file_path = get_share_file('joystick_vehicle_interface_nodes',
    #                                            'launch/joystick_vehicle_interface_node.launch.py')
    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(joystick_launch_file_path),
    #     launch_arguments={
    #         "joy_translator_param": LaunchConfiguration("joy_translator_param"),
    #         "control_command": LaunchConfiguration('control_command')
    #     }.items()
    # )

    ld = LaunchDescription([
        record_traj
        # joy_translator_param,
        # joystick
    ])
    return ld
