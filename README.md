# ur10_teleop_container

## [Host] Container Setup
In host computer,
```
./launch_docker.sh
```

To start and attach to the container
```
docker start ros2_jazzy
docker attach ros2_jazzy
```
To open an additional terminal for container:
```
docker exec -it ros2_jazzy bash
```

## [Container] Build and Source ROS Workspace

To build and source ROS workspace
```
uw # move to workspace
cb # build
sd # source
```

## [Container] Teleoperation with Gazebo

Execute UR10 gazebo simulation (terminal 1)
```
ursim
```

Execute launch file for gazebo teleoperation (terminal 2)
```
tig
```

## [Container] Teleoperation with Real (Mocked) Hardware 

Execute UR10 mocked hardware (terminal 1)
```
urmock
```

Execute launch file for real teleoperation (terminal 2)
```
tir
```

## Shortcuts in Terminal

|Shortcut|Command|Remarks|
|---|---|---|
| eb | gedit ~/.bashrc |
| sb | source ~/.bashrc |
| cb| colcon build --symlink-install --parallel-workers $(nproc) | build the workspace|
| sd | source install/setup.bash | source |
| uw | cd ~/share/ur_ws && source install/setup.bash |
| ka | ros2 daemon stop && sleep 2 && ros2 daemon start && ros2 node list | kill the all nodes |
| conlist| ros2 control list_controllers | print controller status
| nodelist| ros2 node list |
| topiclist| ros2 topic list |
| clearws| rm -rf build install log |
| ursim | ros2 launch ur_simulation_gz ur_sim_control.launch.py |
| urmock | ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=xxx.xxx.xxx.xxx use_mock_hardware:=true'|
| tig | ros2 launch ur10_interface teleop_interface_gazebo.launch.py | teleop interface for gazebo
| tir | ros2 launch ur10_interface teleop_interface_real.launch.py | teleop interface for real ur10
| rqtcm | ros2 run rqt_controller_manager rqt_controller_manager --force-discover |
| msa | ros2 launch moveit_setup_assistant setup_assistant.launch.py | moveit setup assistant

