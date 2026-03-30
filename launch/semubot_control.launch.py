#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=''
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('semubot_description'),  
                'urdf',
                'semubot.urdf.xacro'
            ]),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('semubot_ros_control'),
        'config',
        'semubot_controllers.yaml',
    ])

    joy_config = PathJoinSubstitution([
        FindPackageShare('semubot_ros_control'),
        'config',
        'joy_config.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        remappings=[
            ('/omni_velocity_controller/cmd_vel', '/cmd_vel'),
            ('/omni_velocity_controller/odom', '/odom'),
            ('/diagnostics', '/controller_manager/diagnostics'),
        ]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_velocity_controller', '--controller-manager', '/controller_manager'],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'device_id': 0}],
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[joy_config, {'publish_stamped_twist': False}],
        output='screen'
    )

    delay_omni_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_controller_spawner],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_omni_controller_after_joint_state,
        joy_node,
        teleop_node,
    ])