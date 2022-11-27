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

    pid_params = get_share_file('pid_velocity_control','param/pid.param.yaml')
    # -------------------------------- Nodes-----------------------------------

  

    robot_steering_converter = Node(
        package="robot_steering_converter",
        node_executable="robot_steering_converter_main.py",
        # node_namespace="control",
        node_name="robot_steering_converter",
        output="screen",
        # remappings=[
        #     ("vehicle_command","/vehicle_command")
        # ]
    )
    pid_controller = Node(
        package="pid_velocity_control",
        node_executable="pid_velocity_control_exe",
        node_name="pid_velocity_control",
        output="screen",
        parameters=[
            {"kp":  [1.0,1.0,1.0,1.0]},
            {"kd": [0.0,0.0,0.0,0.0]},
            {"ki": [0.0,0.0,0.0,0.0]},
            {"gain_steps": [100.0,101.0,102.0,103.0]},
            {"max_throttle_acc": 1000.0},
            {"min_throttle_acc": -1000.0},
            {"max_brake_acc": 1000.0},
            {"min_brake_acc": -1000.0},
            {"max_throttle": 10.0},
            {"min_throttle": -10.0},
            {"vrx_simulator": False},

        ],
        # parameters=[{"trajectory_file_name": traj_file_name}],# str(param_file)  {"trajectory_file_name": traj_file_name}

        # parameters=[pid_params],#

        remappings=[
            ("vehicle_state", "vehicle_kinematic_state"),        
        ]
    )

    ld = LaunchDescription([
        robot_steering_converter,
	    pid_controller,
    ])
    return ld
