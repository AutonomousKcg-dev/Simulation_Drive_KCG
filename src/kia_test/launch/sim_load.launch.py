
from inspect import Arguments
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from launch.conditions import IfCondition
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    # --------------------------------- Params -------------------------------

    lexus_urdf_path = get_share_file(
    package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')


    # -------------------------------- Nodes-----------------------------------

    rviz_cfg_path = get_share_file("kia_test", 'config/kia.rviz')
    minimal_simulator_pkg_prefix = get_package_share_directory("minimal_simulator")



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
        # namespace="simulator/ego_vehicle",
        name="minimal_simulator_node",
        output="screen",        
        # LaunchConfiguration("minimal_simulator_param_file")
        parameters=[os.path.join(minimal_simulator_pkg_prefix, "param/vehicles/ego_vehicle_tf.param.yaml")],
        remappings=[
            # ("vehicle_state", "/vehicle_state"),
            #("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/raw_command"),
            #("control_diagnostic", "/control/control_diagnostic"),
        ],
        on_exit=Shutdown()
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
        sync_mode_param,
        sync,
        minimal_simulator
    ])
    return ld
