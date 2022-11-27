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
def get_param_file(package_name, file_name):
    """Helper function to get param file"""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return LaunchConfiguration(
        'params', default=[file_path])

def generate_launch_description():
    traj_file_name1 = get_share_file('kia_test', 'param/trajectories/trajectory_test_1.txt')
    traj_file_name2 = get_share_file('kia_test', 'param/trajectories/trajectory_test_2.txt')

    # --------------------------------- Params -------------------------------
    pid_param_file = get_share_file(
        package_name='pid_velocity_control', file_name='param/pid_kia.param.yaml')

    # --------------------------------- Arguments -------------------------------
    pid_param = DeclareLaunchArgument(
        'pid_param_file',
        default_value=pid_param_file,
        description='Path to config file for PID controller'
    )


    # -------------------------------- Nodes----------------------------------#


    file_trajectory_planner = Node(
        package="file_trajectory_planner",
        executable="file_trajectory_planner_exe",
        name="file_trajectory_planner",
        output="screen",
        parameters=[{"trajectory_file_name1": traj_file_name1},
                    {"trajectory_file_name2": traj_file_name2},
                    {"compute_velocity": False}],
        remappings=[
            ("/vehicle_kinematic_state","vehicle_state"),
            ("trajectory", "trajectory"),
        ]
    )
    
    trajectory_following = Node(
        package="trajectory_following",
        executable="trajectory_following_exe",
        name="trajectory_following",
        output="screen",
        remappings=[
            ("/vehicle_kinematic_state","vehicle_state"),
            ("ctrl_cmd", "/vehicle_command"),
            ("trajectory", "trajectory"),
        ]
    )
    
    # we fricked here
    switch = Node(
        package="controller_pkg",
        executable="switch",
        name="switch",
        output="screen"
    )
    
    controller = Node(
        package="controller_pkg",
        executable="controller",
        name="controller",
        output="screen"
    )

    pid_controller = Node(
        package="pid_velocity_control",
        executable="pid_velocity_control_exe",
        name="pid_velocity_control",
        output="screen",
        parameters=[LaunchConfiguration('pid_param_file')],
        remappings=[
            ("/vehicle_kinematic_state","vehicle_state"),        
        ]
    )
    # and finished fricking here

   

    ld = LaunchDescription([
        pid_param,
        file_trajectory_planner,
        trajectory_following,
        pid_controller,
        switch, # this is fricked
        controller,
    ])
    return ld
