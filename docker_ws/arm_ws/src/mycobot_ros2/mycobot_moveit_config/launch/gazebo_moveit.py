import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mycobot_gazebo')
    pkg_moveit = get_package_share_directory('mycobot_moveit_config')
    
    urdf_path = os.path.join(pkg_gazebo, 'urdf', 'ros2_control', 'classic_gazebo', 'mycobot_280_gazebo_classic.urdf.xacro')
    
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_path]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )
    
    # 1. Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # 3. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mycobot_280', '-topic', 'robot_description', '-z', '0.2'],
        output='screen'
    )

    # 4. Controller Spawners
    # We define them here but trigger them later
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )
    
    # 5. Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )
    
    # 6. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    # --- EVENT HANDLERS (The Fix) ---
    # Start Joint State Broadcaster after the robot is spawned
    load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_jsb],
        )
    )

    # Start Arm Controller after the Broadcaster is finished
    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_arm_controller],
        )
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_jsb,
        load_arm_controller,
        TimerAction(period=10.0, actions=[move_group_node, rviz_node]),
    ])
