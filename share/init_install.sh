sudo apt update
sudo apt install -y python3-pip gedit
pip install --break-system-packages pygame pyyaml

# ROS packages
# ros2_control
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# rqt_controller_manager
sudo apt install -y ros-jazzy-rqt-controller-manager
# install ur dependencies - https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html
sudo apt-get install -y ros-jazzy-ur
# moveit
sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-py  ros-jazzy-moveit-visual-tools
# 
sudo apt install -y ros-jazzy-tf-transformations