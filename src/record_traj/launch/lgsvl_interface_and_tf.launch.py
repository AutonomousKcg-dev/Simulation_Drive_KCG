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
    """
    Launch necessary dependencies for working with LGSVL simulator and ROS 2/Autoware.Auto.

    The LGSVL interface, which translates inputs and outputs to and from ROS standard coordinate
    systems, and the ros2 web bridge, which allows LGSVL to pick up ROS 2 topics.
    """
    # --------------------------------- Params -------------------------------
    lgsvl_interface_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/lgsvl_interface.param.yaml')

    urdf_path = get_share_file('il15_description','urdf/il15.urdf')

    lexus_urdf_path = get_share_file(
    package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')
    # -------------------------------- Nodes-----------------------------------

    rviz_cfg_path = get_share_file("minimal_simulator", 'config/minimal_simulator.rviz')


    lgsvl_interface = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        # node_namespace='ego_vehicle',
        output='screen',
        parameters=[lgsvl_interface_param_file],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            #("vehicle_odom", "/lgsvl/vehicle_odom"),
            ("/vehicle_command", "raw_command")
        ]
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        #node_namespace='vehicle2',
        arguments=[str(lexus_urdf_path)]
        #arguments=[str(lexus_urdf_path)]
        
    )

    static_transform = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        # arguments=['-d', str(minimal_simulator_param)],
    )

    ld = LaunchDescription([
        urdf_publisher,
        static_transform,
        lgsvl_interface,
        rviz2
    ])
    return ld
