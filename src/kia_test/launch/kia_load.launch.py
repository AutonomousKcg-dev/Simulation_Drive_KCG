
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

    lexus_urdf_path = get_share_file(
        package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')

    # -------------------------------- Nodes-----------------------------------

    rviz_cfg_path = get_share_file("kia_test", 'config/kia.rviz')

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[str(lexus_urdf_path)]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
    )

    tf_map_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_gps',
        arguments=["0", "0", "0", "0", "0", "0", "map", "gps_origin"]
    )

    read_gsof = Node(
        package="applanix_driver",
        executable="read_gsof_exe",
        name="read_gsof",
        output="screen",
    )

    parse_gsof = Node(
        package="applanix_driver",
        executable="parse_gsof_exe",
        name="parse_gsof",
        output="screen",
    )

    can_communication = Node(
        package="idan_driver",
        executable="can_communication_titan_exe",
        # node_namespace="control",
        name="can_communication",
        output="screen",
        # parameters=[{"borders_file_name": borders_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}
        # remappings=[
        #     # ("vehicle_command", "raw_command"),
        # ]
    )

    idan_sender = Node(
        package="idan_driver",
        executable="sender_exe",
        # node_namespace="control",
        name="idan_sender",
        output="screen",
        # parameters=[{"borders_file_name": borders_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}
        # remappings=[
        #     # ("vehicle_command", "raw_command"),
        # ]
    )

    idan_listener = Node(
        package="idan_driver",
        executable="listener_exe",
        # node_namespace="control",
        name="idan_listener",
        output="screen",
        # parameters=[{"borders_file_name": borders_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}
        # remappings=[
        #     # ("vehicle_command", "raw_command"),
        # ]
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
        urdf_publisher,
        rviz2,
        # joy_translator_param,
        # joystick
        tf_map_gps,
        read_gsof,
        parse_gsof,
        can_communication,
        idan_sender,
        idan_listener
    ])
    return ld
