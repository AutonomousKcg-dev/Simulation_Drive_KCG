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
    # general_controller_pkg_prefix = get_package_share_directory("general_controller")
    # rviz_cfg_path = os.path.join(general_controller_pkg_prefix, 'config/general_controller_example.rviz')

    # urdf_pkg_prefix = get_package_share_directory('il15_description')
    # urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/il15.urdf')

    # lexus_urdf_path = get_share_file(
    # package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')
    # urdf_path = '/home/gabriel/adehome/AutowareAuto/install/il15_description/share/il15_description/urdf/il15.urdf'
    # In combination 'raw', 'basic' and 'high_level' control
    # in what mode of control comands to operate in,
    # only one of them can be active at a time with a value

    # minimal_simulator_launch_file_path = get_share_file('minimal_simulator',
    #                                         'launch/minimal_simulator.launch.py')

    # multi_simulator_launch_file_path = get_share_file('minimal_simulator',
    #                                         'launch/multi_simulator.launch.py')

    traj_file_name = get_share_file('record_traj', 'param/test.txt')

    # traj_file_name_param = DeclareLaunchArgument(
    #     'traj_file_name_param',
    #     default_value=traj_file_name,
    #     description='trajectory'
    #)
    # param_file = get_share_file(
    #     package_name='file_trajectory_planner', file_name='param/traj.param.yaml')
    # -------------------------------- Nodes-----------------------------------

    # LGSVL interface:

    #
    #minimal simulator:
    # minimal_simulator = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(minimal_simulator_launch_file_path)
    # )
    file_trajectory_planner = Node(
        package="file_trajectory_planner",
        node_executable="file_trajectory_planner_exe",
        # node_namespace="control",
        node_name="file_trajectory_planner",
        output="screen",
        parameters=[{"trajectory_file_name": traj_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}
        remappings=[
            ("vehicle_state", "/ego_vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            # ("trajectory", "/ego_vehicle/vehicle_command"),
        ]
    )
    # sim_perception = Node(
    #     package="sim_perception",
    #     node_executable="sim_perception_main.py",
    #     # node_namespace="control",
    #     node_name="sim_perception",
    #     output="screen",
    #     # remappings=[
    #     #     ("vehicle_state", "/ego_vehicle/vehicle_kinematic_state"),
    #     #     ("ctrl_cmd", "/ego_vehicle/vehicle_command"),
    #     # ]
    # )

    # rviz2 = Node(
    #     package='rviz2',
    #     node_executable='rviz2',
    #     node_name='rviz2',
    #     arguments=['-d', str(rviz_cfg_path)],
    # )
    # urdf_publisher = Node(
    #     package='robot_state_publisher',
    #     node_executable='robot_state_publisher',
    #     node_name='robot_state_publisher',
    #     #node_namespace='vehicle2',
    #     #arguments=[str(urdf_path)]
    #     arguments=[str(lexus_urdf_path)]
        
    # )
    # urdf_publisher2 = Node(
    #     package='robot_state_publisher',
    #     node_executable='robot_state_publisher',
    #     node_name='robot_state_publisher',
    #     node_namespace='vehicle2',
    #     arguments=[
    #         str(urdf_path),
    #         'tf_prefix:=dddd',]
        #arguments=[str(lexus_urdf_path)]
        
    # )
    # ros2 web bridge
    # lgsvl_bridge = launch.actions.ExecuteProcess(cmd=["rosbridge"], shell=True)

    ld = LaunchDescription([
        # minimal_simulator,
        file_trajectory_planner,
        # sim_perception,
        # rviz2,
        # urdf_publisher,
        # urdf_publisher2
    ])
    return ld
