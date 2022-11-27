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
    # traj_file_name = get_share_file('file_trajectory_planner', 'param/trajectories/full_trajectory.txt')


    # --------------------------------- Params -------------------------------

    # -------------------------------- Nodes-----------------------------------



    read_gsof = Node(
        package="applanix_driver",
        node_executable="read_gsof_exe",
        # node_namespace="control",
        name="read_gsof",
        output="screen",
        # parameters=[{"borders_file_name": borders_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}
        # remappings=[
        #     # ("vehicle_command", "raw_command"),
        # ]
    )

    parse_gsof = Node(
        package="applanix_driver",
        node_executable="parse_gsof_exe",
        # node_namespace="control",
        name="parse_gsof",
        output="screen",
        # parameters=[{"borders_file_name": borders_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}
        # remappings=[
        #     # ("vehicle_command", "raw_command"),
        # ]
    )


    ld = LaunchDescription([
        read_gsof,
        parse_gsof
    ])
    return ld
