import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    traj_file_name = get_share_file(
        'kia_test', 'param/trajectories/trajectory_test_1.txt')

    return LaunchDescription([
        Node(
            package='record_traj',
            executable='record_traj_exe',
            # node_namespace='ego_vehicle',
            output='screen',
            parameters=[{"record_file": traj_file_name}
                        ],
            remappings=[
                ("/vehicle_kinematic_state", "vehicle_state")
            ]
        ),
        # Node(
        #     package='cognata_pkg',
        #     executable='cognata_listener'
        # ),
        Node(
            package='cognata_pkg',
            executable='cognata_raw_command'
        )
    ])
