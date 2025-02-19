# ur10_teleop_container
[Host]
1. Generate docker container
    ```
    ./launch_docker.sh
    ```
[In Container]

2. Build and source ROS workspace
    ```
    uw && cb && sd
    ```
3. Execute UR10 gazebo simulation (terminal 1)
    ```
    usc
    ```

4. Execute teleop nodes (terminal 2)
    ```
    ti
    ```
