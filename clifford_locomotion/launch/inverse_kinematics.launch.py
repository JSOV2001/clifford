# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

locomotion_pkg_filepath = get_package_share_directory("clifford_locomotion")

robot_state_publisher_launch_filepath = os.path.join(
    locomotion_pkg_filepath, 
    "launch",
    "robot_state_publisher.launch.py"
)

rviz_config_filepath = os.path.join(
    locomotion_pkg_filepath,
    "config",
    "inverse_kinematics.rviz"
)

def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher_launch_filepath]),
        launch_arguments= {
            'use_sim_time': 'true'
        }.items()
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_config_filepath]
    )

    inverse_kinematics_publisher_node = Node(
        package= "clifford_locomotion",
        executable= "inverse_kinematics_publisher.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

    nodes_to_run = [
        robot_state_publisher_launch, 
        rviz_cmd, 
        inverse_kinematics_publisher_node
    ]
    return LaunchDescription(nodes_to_run)