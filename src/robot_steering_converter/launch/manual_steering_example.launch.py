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
    general_controller_pkg_prefix = get_package_share_directory("general_controller")
    rviz_cfg_path = get_share_file('robot_steering_converter', 'config/manual_example.rviz')

    urdf_pkg_prefix = get_package_share_directory('il15_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/il15.urdf')

    lexus_urdf_path = get_share_file(
    package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')

    pid_params = get_share_file('pid_velocity_control','param/pid.param.yaml')

    # urdf_path = '/home/gabriel/adehome/AutowareAuto/install/il15_description/share/il15_description/urdf/il15.urdf'
    # In combination 'raw', 'basic' and 'high_lros2 param listevel' control
    # in what mode of control comands to operate in,
    # only one of them can be active at a time with a value

    minimal_simulator_launch_file_path = get_share_file('minimal_simulator',
                                            'launch/minimal_simulator.launch.py')
    # -------------------------------- Nodes-----------------------------------

    #minimal simulator:
    minimal_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(minimal_simulator_launch_file_path)
    )

    robot_steering_converter = Node(
        package="robot_steering_converter",
        node_executable="robot_steering_converter_main.py",
        # node_namespace="control",
        node_name="robot_steering_converter",
        output="screen",
        remappings=[
            ("vehicle_command","ego_vehicle/vehicle_command")
        ]
    )
    pid_controller = Node(
        package="pid_velocity_control",
        node_executable="pid_velocity_control_exe",
        node_namespace="ego_vehicle",
        node_name="pid_velocity_control",
        output="screen",
        parameters=[
            {"kp":  [10.0,1.0,1.0,1.0]},
            {"kd": [0.0,0.0,0.0,0.0]},
            {"ki": [0.0,0.0,0.0,0.0]},
            {"gain_steps": [100.0,101.0,102.0,103.0]},
            {"max_throttle_acc": 1000.0},
            {"min_throttle_acc": -1000.0},
            {"max_brake_acc": 1000.0},
            {"min_brake_acc": -1000.0},
            {"max_throttle": 10.0},
            {"min_throttle": -10.0},
        ],
        # parameters=[{"trajectory_file_name": traj_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}

        # parameters=[pid_params],#

        remappings=[
           ("vehicle_state", "vehicle_kinematic_state"),        
        ]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        #arguments=[str(urdf_path)]
        arguments=[str(lexus_urdf_path)]
        
    )
    # ros2 web bridge
    # lgsvl_bridge = launch.actions.ExecuteProcess(cmd=["rosbridge"], shell=True)

    ld = LaunchDescription([
        minimal_simulator,
        robot_steering_converter,
	pid_controller,
        rviz2,
        urdf_publisher
    ])
    return ld
