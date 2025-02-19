import os
from sys import prefix
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    ## MoveIt settings
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur10_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # scaled_joint_trajectory_controller
        .moveit_cpp(
            file_path=get_package_share_directory("ur10_interface")
            + "/config/motion_planning_parameters.yaml"
        )
        .to_moveit_configs()
    )    
    move_group_launch = generate_move_group_launch(moveit_config)
    
    ## Nodes
    input_node = Node(
        package="ur10_interface",
        executable="input_command.py",
        output="log",
    )
    mode_manager_node = Node(
        package="ur10_interface",
        executable="mode_manager.py",
        output="log",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}, # to match time stamp of "/joint_states" topic with gazebo
        ],
        arguments=['--env', 'gazebo'], # to load environment specific config file
    )
    delta_input_node = Node(
        package="ur10_interface",
        executable="delta_target_input.py",
        output="log",
        arguments=['--env', 'gazebo'], # to load environment specific config file
    )
    
    target_pose_node = Node(
        package="ur10_interface",
        executable="target_pose.py",
        output="log",
        arguments=['--env', 'gazebo'], # to load environment specific config file
    )
    
    task2joint_node = Node(
        package="ur10_interface",
        executable="task2joint",
        output="log",
        parameters=[moveit_config.to_dict()],
    )
    
    teleop_controller_node = Node(
        package="ur10_interface",
        executable="teleop_controller.py",
        output="log",
    )
    
    visualize_target_pose_node = Node(
        package="ur10_interface",
        executable="visualize_target_pose.py",
        output="log",
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True), # set "use_sim_time" to true for all instances in the launch file
        move_group_launch,
        input_node,
        mode_manager_node,
        # TimerAction(
        #     period=5.0,
        #     actions=[delta_input_node, target_pose_node, task2joint_node, teleop_controller_node, visualize_target_pose_node],
        # )
        delta_input_node,
        target_pose_node,
        task2joint_node,
        teleop_controller_node,
        visualize_target_pose_node,
    ])