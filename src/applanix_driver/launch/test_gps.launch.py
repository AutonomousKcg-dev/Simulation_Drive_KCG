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
    rviz_cfg_path = os.path.join( get_package_share_directory("applanix_driver"), 'config/test_gps.rviz')

    # urdf_pkg_prefix = get_package_share_directory('il15_description')
    # urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/il15.urdf')
    lexus_urdf_path = get_share_file(package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')
    
    tf_map_gps = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        name='tf_map_gps',
        # arguments=["-31.051352", "19.317709", "700.176012", "0", "0", "0", "map", "gps_origin"] #-3.072896
        arguments=["0", "0", "0", "0", "0", "0", "map", "gps_origin"]

         
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
        # arguments=[str(urdf_path)]
        arguments=[str(lexus_urdf_path)]
    )


    ld = LaunchDescription([
        tf_map_gps,
        rviz2,
        urdf_publisher
    ])
    return ld
