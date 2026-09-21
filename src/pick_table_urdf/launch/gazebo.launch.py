import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    pick_table_share = get_package_share_directory("pick_table_urdf")

    urdf_path = os.path.join(
        pick_table_share,
        "urdf",
        "pick_table_urdf.urdf",
    )

    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=(
            os.path.dirname(pick_table_share)
            + ":"
            + os.environ.get("GAZEBO_MODEL_PATH", "")
        ),
    )

    gazebo = IncludeLaunchDescription(
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
        output="screen",
        arguments=[
            "-entity",
            "pick_table_urdf",
            "-file",
            urdf_path,
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        spawn_table,
    ])
