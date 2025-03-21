cmake_minimum_required(VERSION 3.8)
project(ur10_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # for python scripts
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED) # for custom messages
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_py REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# include cpp "include" directory
include_directories(include)

# create cpp executable
add_executable(task2joint src/task2joint.cpp)
ament_target_dependencies(task2joint rclcpp std_msgs moveit_ros_planning_interface tutorial_interfaces)
add_executable(task2joint_real src/task2joint_real.cpp)
ament_target_dependencies(task2joint_real rclcpp std_msgs moveit_ros_planning_interface tutorial_interfaces)

# install cpp executable
# install(TARGETS talker
#   DESTINATION lib/${PROJECT_NAME})
install(TARGETS task2joint
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS task2joint_real
  DESTINATION lib/${PROJECT_NAME})

# install python modules
ament_python_install_package(${PROJECT_NAME}) # rosidl_generate_interfaces에서 중복 발생

# install python executables
install(PROGRAMS
  scripts/current_ee_pose.py
  scripts/input_command.py
  scripts/mode_manager.py
  scripts/delta_target_input.py
  scripts/target_pose.py
  scripts/teleop_controller.py
  scripts/visualize_target_pose.py
  scripts/tool_gravity.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
