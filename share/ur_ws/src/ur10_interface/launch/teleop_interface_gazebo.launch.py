import os
from sys import prefix
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.launch_description_sources import PythonLaunchDescriptionSource

def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument("ur_type", default_value="ur10", description="Typo/series of used UR robot."),
            DeclareLaunchArgument("env", default_value="gazebo", description="Environment type: real or gazebo"),
        ]
    )

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    env = LaunchConfiguration("env")
    
    ## MoveIt settings   
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .moveit_cpp(
            file_path=get_package_share_directory("ur10_interface")
            + "/config/motion_planning_parameters.yaml"
        )
        .to_moveit_configs()
    )
    
    ## Launchs
    # movit launch
    pkg_name = "ur_moveit_config"
    other_launch_file = os.path.join(
        get_package_share_directory(pkg_name), "launch", "ur_moveit_launch.py"
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ur_type': ur_type,
        }.items()
    )
    
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
            {'use_sim_time': use_sim_time}, # to match time stamp of "/joint_states" topic with gazebo
        ],
        arguments=['--env', env], # to load environment specific config file
    )
    delta_input_node = Node(
        package="ur10_interface",
        executable="delta_target_input.py",
        output="log",
        arguments=['--env', env], # to load environment specific config file
    )
    
    target_pose_node = Node(
        package="ur10_interface",
        executable="target_pose.py",
        output="screen",
        arguments=['--env', env], # to load environment specific config file
    )
    
    task2joint_node = Node(
        package="ur10_interface",
        executable="task2joint",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
        
    teleop_controller_node = Node(
        package="ur10_interface",
        executable="teleop_controller.py",
        output="log",
        arguments=['--env', env], # to load environment specific config file
    )
    
    visualize_target_pose_node = Node(
        package="ur10_interface",
        executable="visualize_target_pose.py",
        output="log",
    )


    return LaunchDescription([
        declare_arguments(),
        SetParameter(name='use_sim_time', value=use_sim_time), # set "use_sim_time" to true for all instances in the launch file
        #move_group_launch,
        input_node,
        mode_manager_node,
        delta_input_node,
        target_pose_node,
        task2joint_node,
        teleop_controller_node,
        visualize_target_pose_node,
    ])