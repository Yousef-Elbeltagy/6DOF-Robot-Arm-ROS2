import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    robot_package = get_package_share_directory('robot_arm_with_gripper_urdf')
    table_package = get_package_share_directory('table_harness_urdf_v3')
    pick_table_package = get_package_share_directory('pick_table_urdf')
    large_jig_package = get_package_share_directory('large_jig_urdf')
    medium_jig_package = get_package_share_directory('medium_jig_urdf')
    small_jig_package = get_package_share_directory('small_jig_urdf')
    gazebo_package = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(
        robot_package,
        'urdf',
        'robot_arm_with_gripper_urdf.urdf'
    )

    table_urdf_path = os.path.join(
        table_package,
        'urdf',
        'table_harness_urdf_v3.urdf'
    )

    pick_table_urdf_path = os.path.join(
        pick_table_package,
        'urdf',
        'pick_table_urdf.urdf'
    )

    large_jig_urdf_path = os.path.join(
        large_jig_package,
        'urdf',
        'large_jig_urdf.urdf'
    )

    medium_jig_urdf_path = os.path.join(
        medium_jig_package,
        'urdf',
        'medium_jig_urdf.urdf'
    )

    small_jig_urdf_path = os.path.join(
        small_jig_package,
        'urdf',
        'small_jig_urdf.urdf'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.dirname(table_package),
            ':',
            os.path.dirname(pick_table_package),
            ':',
            os.path.dirname(large_jig_package),
            ':',
            os.path.dirname(medium_jig_package),
            ':',
            os.path.dirname(small_jig_package),
            ':',
            EnvironmentVariable(
                'GAZEBO_MODEL_PATH',
                default_value=''
            )
        ]
    )
    world_path = os.path.join(
        robot_package,
        'worlds',
        'robot_with_link_attacher.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_package, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )
    table_base_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='table_base_transform',
        arguments=[
            '--x', '0.9',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'base_link',
            '--child-frame-id', 'table_base_link'
        ],
        output='screen'
    )
    m1_target_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='m1_target_transform',
        arguments=[
            '--x', '-0.27578',
            '--y', '0.2314',
            '--z', '0.3525',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'table_base_link',
            '--child-frame-id', 'm1_target_link'
        ],
        output='screen'
    )
    
    m2_target_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='m2_target_transform',
        arguments=[
            '--x', '-0.191511110779744',
            '--y', '-0.160696902421635',
            '--z', '0.3525',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'table_base_link',
            '--child-frame-id', 'm2_target_link'
        ],
        output='screen'
    )
    s1_target_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='s1_target_transform',
        arguments=[
            '--x', '-0.0162000258782906',
            '--y', '-0.000766044443119007',
            '--z', '0.3525',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'table_base_link',
            '--child-frame-id', 's1_target_link'
        ],
        output='screen'
    )

    l1_target_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='l1_target_transform',
        arguments=[
            '--x', '0.35',
            '--y', '0',
            '--z', '0.3525',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'table_base_link',
            '--child-frame-id', 'l1_target_link'
        ],
        output='screen'
    )        
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'table_harness_urdf_v3',
            '-file', table_urdf_path,
            '-timeout', '300',
            '-x', '0.9',
            '-y', '0',
            '-z', '0',
            '-Y', '-1.57079632679'
        ]
    ) 
    spawn_pick_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'pick_table_urdf',
            '-file', pick_table_urdf_path,
            '-timeout', '300',
            '-x', '0',
            '-y', '-0.9',
            '-z', '0',
            '-Y', '3.14159265359'
        ]
    )

    spawn_large_jig_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'large_jig_1',
            '-file', large_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.2005',
            '-y', '-0.7800',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_large_jig_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'large_jig_2',
            '-file', large_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.2005',
            '-y', '-0.8600',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_large_jig_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'large_jig_3',
            '-file', large_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.2005',
            '-y', '-0.9400',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_large_jig_4 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'large_jig_4',
            '-file', large_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.2005',
            '-y', '-1.0200',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_medium_jig_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'medium_jig_1',
            '-file', medium_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.0005',
            '-y', '-0.7800',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_medium_jig_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'medium_jig_2',
            '-file', medium_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.0005',
            '-y', '-0.8600',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_medium_jig_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'medium_jig_3',
            '-file', medium_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.0005',
            '-y', '-0.9400',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_medium_jig_4 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'medium_jig_4',
            '-file', medium_jig_urdf_path,
            '-timeout', '300',
            '-x', '0.0005',
            '-y', '-1.0200',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_small_jig_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'small_jig_1',
            '-file', small_jig_urdf_path,
            '-timeout', '300',
            '-x', '-0.1995',
            '-y', '-0.7800',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_small_jig_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'small_jig_2',
            '-file', small_jig_urdf_path,
            '-timeout', '300',
            '-x', '-0.1995',
            '-y', '-0.8600',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_small_jig_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'small_jig_3',
            '-file', small_jig_urdf_path,
            '-timeout', '300',
            '-x', '-0.1995',
            '-y', '-0.9400',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_small_jig_4 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'small_jig_4',
            '-file', small_jig_urdf_path,
            '-timeout', '300',
            '-x', '-0.1995',
            '-y', '-1.0200',
            '-z', '0.3525',
            '-Y', '4.71238898038'
        ]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot_arm_with_gripper',
            '-topic', 'robot_description',
            '-timeout', '300',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ]
    )
    joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'joint_state_broadcaster',
        '--controller-manager',
        '/controller_manager'
    ],
    output='screen'
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        table_base_transform,
        m1_target_transform,
        m2_target_transform,
        s1_target_transform,
        l1_target_transform,
        spawn_robot,
        spawn_table,
        spawn_pick_table,
        TimerAction(
            period=3.0,
            actions=[
                spawn_large_jig_1,
                spawn_large_jig_2,
                spawn_large_jig_3,
                spawn_large_jig_4,
                spawn_medium_jig_1,
                spawn_medium_jig_2,
                spawn_medium_jig_3,
                spawn_medium_jig_4,
                spawn_small_jig_1,
                spawn_small_jig_2,
                spawn_small_jig_3,
                spawn_small_jig_4,
            ],
        ),
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
