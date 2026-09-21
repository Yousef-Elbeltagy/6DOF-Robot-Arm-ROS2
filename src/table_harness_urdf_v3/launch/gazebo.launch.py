from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    package_share = get_package_share_directory("table_harness_urdf_v3")

    urdf_file = os.path.join(
        package_share,
        "urdf",
        "table_harness_urdf_v3.urdf",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    spawn_table = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "table_harness_urdf_v3",
            "-file",
            urdf_file,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_launch,
            spawn_table,
        ]
    )
