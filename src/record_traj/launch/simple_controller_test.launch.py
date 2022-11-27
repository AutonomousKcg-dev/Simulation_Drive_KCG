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
    traj_file_name = get_share_file('file_trajectory_planner', 'param/trajectories/full_trajectory.txt')
    traj_out_file_name = get_share_file('file_trajectory_planner', 'param/trajectories/full_trajectory_out.txt')

    # --------------------------------- Params -------------------------------
    pid_param_file = get_share_file(
        package_name='pid_velocity_control', file_name='param/pid.param.yaml')

    # --------------------------------- Arguments -------------------------------
    pid_param = DeclareLaunchArgument(
        'pid_param_file',
        default_value=pid_param_file,
        description='Path to config file for PID controller'
    )


    # -------------------------------- Nodes-----------------------------------



    simple_controller = Node(
        package="simple_controller_test",
        node_executable="simple_controller_test_exe",
        name="simple_controller_test",
        output="screen",
        # remappings=[
        #     ("vehicle_state", "vehicle_kinematic_state"),        
        # ]
    )


    ld = LaunchDescription([
        # traj_file_name_param,
        simple_controller

    ])
    return ld
