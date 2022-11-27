from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path
from ament_index_python import get_package_share_directory
import launch

import os

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)



# def get_package_share_directory(package_name):
#     """Return the absolute path to the share directory of the given package."""
#     return os.path.join(path_file
#         Path(FindPackage(package_name).perform(context)), "share", package_name
#     )


def generate_launch_description():
    """Launch controller_testing_node and mpc_controller."""
    minimal_simulator_pkg_prefix = get_package_share_directory("minimal_simulator")
    # minimal_simulator_param_file = os.path.join(
    #     minimal_simulator_pkg_prefix, "param/defaults.param.yaml"
    # )
    # minimal_simulator_il15_sim_param_file = os.path.join(path_file
    #     minimal_simulator_pkg_prefix, "param/il15_sim.param.yaml"
    # )

    # Arguments
    # minimal_simulator_param = DeclareLaunchArgument(
    #     "minimal_simulator_param_file",
    #     default_value=minimal_simulator_il15_sim_param_file,#minimal_simulator_param_file,
    #     description="Path to config file for minimal simulator",
    # )
    # real_time_sim_param = DeclareLaunchArgument(
    #     'real_time_sim',
    #     default_value='False',
    #     description='Launch RVIZ2 in addition to other nodes'
    # )

    rviz_cfg_path = os.path.join(minimal_simulator_pkg_prefix, 'config/minimal_simulator.rviz')

    urdf_pkg_prefix = get_package_share_directory('il15_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/il15.urdf')

    sync_mode_param = DeclareLaunchArgument(
        'auto_sync_mode',
        default_value='False',
        description='simulator sync mode'
    )

    # Nodes
    sync =  Node(
        package="simulation_sync",
        node_executable="simulation_sync_main.py",
        node_name="simulation_sync_node",
        output="screen",        
        remappings=[
            ("vehicle_command", "/raw_command"),
        ],
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('auto_sync_mode'))
    )

    minimal_simulator = Node(
        package="minimal_simulator",
        node_executable="minimal_simulator_main.py",
        node_namespace="simulator/ego_vehicle",
        name="minimal_simulator_node",
        output="screen",        
        # LaunchConfiguration("minimal_simulator_param_file")
        parameters=[os.path.join(minimal_simulator_pkg_prefix, "param/vehicles/option7/ego_vehicle.param.yaml")],
        remappings=[
            ("vehicle_state", "/vehicle_kinematic_state"),
            #("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/raw_command"),
            #("control_diagnostic", "/control/control_diagnostic"),
        ],
        on_exit=Shutdown()
    )
    vehicle_num = 7 #int(launch.substitutions.LaunchConfiguration('vehicle_num'))
    # path_file = os.path.join(
    #         minimal_simulator_pkg_prefix, "param/commands/path"+str(i)+".txt"
    #     )
    # with open(self.param_file_name,'r') as f:

    vehicle_nodes = []
    for i in range(1,vehicle_num+1):
        vehicle_nodes.append(
            Node(
                package="minimal_simulator",
                node_executable="minimal_simulator_main.py",
                node_namespace="simulator/vehicle"+str(i),
                name="minimal_simulator_node",
                output="screen",
                parameters=[os.path.join(minimal_simulator_pkg_prefix, "param/vehicles/option7/vehicle"+str(i)+".param.yaml")
                ],
                remappings=[
                    ("vehicle_state", "/simulator/vehicle"+str(i)+"/vehicle_kinematic_state"),
                    #("planned_trajectory", "/planning/trajectory"),
                    ("control_command", "/simulator/vehicle"+str(i)+"/vehicle_command"),
                    # ("control_command", "/vehicle"+str(i)+"/vehicle_command"),
                    #("control_diagnostic", "/control/control_diagnostic"),
                    #("/tf", "/vehicle2/tf"),
                ],
                # remappings=[
                #     ("vehicle_state", "/vehicle_kinematic_state"),
                #     #("planned_trajectory", "/planning/trajectory"),
                #     ("control_command", "/vehicle_command"),
                #     #("control_diagnostic", "/control/control_diagnostic"),
                #     #("/tf", "/vehicle2/tf"),
                # ],
                on_exit=Shutdown()
            )
        )




    sim_perception = Node(
        package="sim_perception",
        node_executable="sim_perception_main.py",
        node_namespace="simulator/perception",
        name="sim_perception",
        output="screen",
        parameters=[{"vehicle_num": vehicle_num}],
        remappings=[
            ("vehicle_state", "/vehicle_kinematic_state"),
            ("bounding_boxes", "/bounding_boxes"),
        ]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        #node_namespace='vehicle2',
        arguments=[str(urdf_path)]
        #arguments=[str(lexus_urdf_path)]
        
    )


    #####
    file_trajectory_planner_nodes = []
    for i in range(1,vehicle_num+1):
        traj_file_name = get_share_file('file_trajectory_planner', 'param/trajectories/option7/traj'+str(i)+'.txt')
        file_trajectory_planner_nodes.append(
            Node(
                package="file_trajectory_planner",
                node_executable="file_trajectory_planner_exe",
                node_namespace="simulator/file_trajectory_planner"+str(i),
                name="file_trajectory_planner",
                output="screen",
                parameters=[{"trajectory_file_name": traj_file_name},
                            {"trajectory_out_file_name": traj_file_name},
                            {"compute_velocity": False}],# str(param_file)  {"trajectory_file_name": traj_file_name}
                remappings=[
                    ("vehicle_state", "/simulator/vehicle"+str(i)+"/vehicle_kinematic_state"),
                    ("trajectory", "/vehicle"+str(i)+"/planning/trajectory"),
                ],
            )
        )

      
    pure_pursuit_nodes = []
    for i in range(1,vehicle_num+1):
        pure_pursuit_nodes.append(
            Node(
                package="simple_pure_pursuit",
                node_executable="simple_pure_pursuit_exe",
                node_namespace="simulator/simple_pure_pursuit"+str(i),
                name="simple_pure_pursuit",
                output="screen",
                remappings=[
                    ("vehicle_state", "/simulator/vehicle"+str(i)+"/vehicle_kinematic_state"),
                    ("ctrl_cmd", "/simulator/vehicle"+str(i)+"/vehicle_command"),
                    ("trajectory", "/vehicle"+str(i)+"/planning/trajectory"),
                ],
            )
        )  


    return LaunchDescription(
        [
            sync_mode_param,
            sync,
            # real_time_sim_param,
            sim_perception,
            rviz2,
            urdf_publisher,
            #minimal_simulator_param,
            minimal_simulator,
        ]+vehicle_nodes+file_trajectory_planner_nodes+pure_pursuit_nodes
    )
