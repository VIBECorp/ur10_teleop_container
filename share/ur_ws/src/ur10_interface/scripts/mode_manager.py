#!/usr/bin/env python3
# -*- coding: utf8 -*- 
# from ur10_interface.common_utils import *
import argparse
import yaml
import os
import time
import numpy as np
import copy
import json

from ament_index_python.packages import get_package_share_directory

def get_package_dir(package_name):
    share_dir = get_package_share_directory(package_name)
    package_dir = share_dir.replace('install', 'src').removesuffix(f'/share/{package_name}')
    return package_dir

def get_file_dir(package_name, file_name):
    pkg_dir = get_package_dir(package_name)
    file_dir = os.path.join(pkg_dir, file_name)
    return file_dir
    

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
AI = 4
MOVEIT = 5
IDLE = 6
GCOMP = 7
DIRECT = 8

mode_dict = {
    0: "INIT",
    1: "TELEOP",
    2: "TASK_CONTROL",
    3: "JOINT_CONTROL",
    4: "AI",
    5: "MOVEIT",
    6: "IDLE",
    7: "GCOMP",
    8: "DIRECT",
    -1: "READY",
}

axis_dict = {
    0: 'z',
    1: 'x',
    2: 'y'
}

# ROS2 library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Wrench, PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from tool_gravity import ToolGravityCompensation
from scipy.spatial.transform import Rotation as R


def aligned_joint_position(current_joint_pos, axis='z'):
    aligned = current_joint_pos[:]
    init = current_joint_pos[:]
    if axis == 'z':
        # q2 + q3 + q4 = -90
        # q4 = -90 - q2 - q3
        aligned[3] = -(init[1] + init[2]) - np.deg2rad(90.0)
        aligned[4] = -np.deg2rad(90.0)
        aligned[5] = 0.0
        
    elif axis == 'x':
        aligned[3] = -(init[1] + init[2]) - np.deg2rad(90.0)
        aligned[4] = 0.0
        aligned[5] = 0.0
        
    elif axis == 'y':
        aligned[3] = -(init[1] + init[2]) - np.deg2rad(90.0)
        aligned[4] = 0.0
        aligned[5] = -np.deg2rad(90.0)
        
    return aligned



def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)
    logger.info("Completed execution")

class ModeManager(Node):
    def __init__(self, args, ur10_moveit_py):
        super().__init__('mode_manager_node')
        
        # load parameters
        self.load_parameters_from_config(args)
        self.button = 0.0
        self.mode = -1 # self.config['mode']
        self.base_controller = self.config['base_controller']
        self.velocity_controller = self.config['velocity_controller']
        self.planning_group = self.config['planning_group']
        # self.init_joint_states = self.config['init_joint_states']
        self.current_pose = None
        self.run_gcomp = False
        self.current_joint_states = None
        self.init_joint_states = None
        self.target_joint_pos = None
        self.current_ft_data = [0.0] * 6
        self.current_index = 0
        self.current_axis = axis_dict[self.current_index]
        self.start_saving = False
        self.saved_ft = {}
        self.saved_tf_ft = {}
        self.ft_saving_path = get_file_dir('ur10_interface', self.config['ft_raw_save_to'])
        self.tf_ft_saving_path = get_file_dir('ur10_interface', self.config['tf_ft_save_to'])
        self.zero_ft_saving_path = get_file_dir('ur10_interface', self.config['ft_zero_save_to'])
        self.tool_cg_path = get_file_dir('ur10_interface', self.config['tool_cg_saved'])
        self.tool_cg = None
        
        # planning component
        self.ur10_moveit_py = ur10_moveit_py
        self.ur10_arm = ur10_moveit_py.get_planning_component(self.planning_group)
        self.robot_model = ur10_moveit_py.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        self.get_logger().info(f"MoveItPy instance created, {type(self.ur10_arm)}")

        # publisher & subscriber
        self.mode_pub = self.create_publisher(Int32, 'mode', 10)
        self.joystick_command_sub = self.create_subscription(Float64MultiArray, 'joystick_command', self.joystick_command_callback, 10)
        self.keyboard_command_sub = self.create_subscription(Float64MultiArray, 'keyboard_command', self.keyboard_command_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_joint_pos_sub = self.create_subscription(Float64MultiArray, 'target_joint_position', self.target_joint_pos_callback, 10)
        self.ft_wrench_sub = self.create_subscription(Wrench, self.config['ft_sensor_node'], self.ft_data_callback, 10)
        # self.tf_ft_sub = self.create_subscription(Wrench, self.config['tf_ft_node'], self.tf_ft_data_callback, 10)
        self.create_subscription(PoseStamped, 'current_ee_pose', self.current_pose_callback, 10)
        
        # service
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers')

        # initialize pose
        self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, INIT)])
        # self.change_to_base_controller()
        # # delay
        # time.sleep(2.0)
        # self.move_to_config_pose('init', sleep_time=3.0)

        # mode loop
        self.timer = self.create_timer(0.001, self.loop)
        
    def current_pose_callback(self, msg):
        self.current_pose = msg
        
    
    def ft_data_callback(self, msg):
        self.current_ft_data[0] = msg.force.x
        self.current_ft_data[1] = msg.force.y
        self.current_ft_data[2] = msg.force.z
        self.current_ft_data[3] = msg.torque.x
        self.current_ft_data[4] = msg.torque.y
        self.current_ft_data[5] = msg.torque.z
        # self.get_logger().info(f"FT: {self.current_ft_data}")
        
        tf_FT = [0, 0, 0, 0, 0, 0]
        if self.current_pose is None:
            pass
        else:
            qx = self.current_pose.pose.orientation.x
            qy = self.current_pose.pose.orientation.y
            qz = self.current_pose.pose.orientation.z
            qw = self.current_pose.pose.orientation.w
            
            quat = [qx, qy, qz, qw]
            
            rot = R.from_quat(quat).as_matrix()
            
            F = np.array(self.current_ft_data[:3])
            T = np.array(self.current_ft_data[3:])
            
            # rF = rot.T @ F
            # rT = rot.T @ T
            rF = rot @ F
            rT = rot @ T
            tf_FT = [rF[0], rF[1], rF[2], rT[0], rT[1], rT[2]]

        if self.start_saving:
            if len(self.saved_ft[self.current_axis]) < self.config['ft_num_measure']:
                self.saved_ft[self.current_axis].append(self.current_ft_data[:])
                self.saved_tf_ft[self.current_axis].append(tf_FT)
            else:
                self.get_logger().info(f"Current axis: {self.current_axis}\n Num saved: {len(self.saved_ft[self.current_axis])}")
                self.current_index += 1
                self.start_saving = False
            
    def save_ft_raw(self):
        with open(self.ft_saving_path, 'w') as f:
            json.dump(self.saved_ft, f)
            
        with open(self.tf_ft_saving_path, 'w') as f:
            json.dump(self.saved_tf_ft, f)
            
        self.get_logger().info(f"Raw FT data is saved to {self.ft_saving_path}")
    
    
    def target_joint_pos_callback(self, msg):
        self.target_joint_pos = msg.data
        
        
    def joint_state_callback(self, msg):
        temp = list(msg.position)  # 새로운 리스트로 변환하여 복사
        temp2 = temp[:]  # 깊은 복사 (Shallow Copy)

        # 인덱스 변경
        temp2[0] = temp[5]
        temp2[1] = temp[0]
        temp2[2] = temp[1]
        temp2[3] = temp[2]
        temp2[4] = temp[3]
        temp2[5] = temp[4]
        self.current_joint_states = temp2
        # self.init_joint_states = list(self.current_joint_states)
        # self.get_logger().info(f"Current joint states: {self.current_joint_states}")
    
        
    def load_parameters_from_config(self, args):
        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Upload parameters
        for key, value in self.config.items():
            self.declare_parameter(key, value)
            # self.get_logger().info(f"Loaded param: {key} = {value}")
        
    def joystick_command_callback(self, msg):
        self.joystick_command = msg.data
        self.button = self.joystick_command[-1]
    
    def keyboard_command_callback(self, msg):
        self.keyboard_command = msg.data
        self.button = self.keyboard_command[-1]
        

    def loop(self):
        if self.current_joint_states is None:
            self.get_logger().info("Joint states is None. Waiting...")
            return
        #self.get_logger().info(f"Mode: {mode_dict[self.mode]}")
        # mode change by input
        if self.button == 6.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, INIT)])
        elif self.button == 7.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, TELEOP)])
        elif self.button == 12.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, GCOMP)])
        elif self.button == 13.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, DIRECT)])
        # read current mode
        mode = self.get_parameter('mode').get_parameter_value().integer_value
        self.mode_pub.publish(Int32(data=mode))
        
        # if mode changed
        if mode is not self.mode:
            self.get_logger().info(f"Mode changed from {mode_dict[self.mode]} to {mode_dict[mode]}")
            # switch controller interface
            if mode in [INIT, MOVEIT, GCOMP]:
                self.change_to_base_controller()
                if mode == GCOMP:
                    self.set_current_pose_as_init()
                    self.get_logger().info("Gravity compensation mode")
                    self.run_gcomp = True
                else:
                    self.run_gcomp = False
                    
            elif mode in [TELEOP, TASK_CONTROL, JOINT_CONTROL, IDLE, AI, DIRECT]:
                self.run_gcomp = False
                self.change_to_velocity_controller()

            # move pose
            if mode == INIT:
                self.set_current_pose_as_init()
                self.move_to_config_pose('init')
                self.get_logger().info("Init pose")
                
            self.mode = mode
                
                
        if mode == GCOMP and not self.current_joint_states is None and self.run_gcomp:
            
            if self.start_saving:
                # self.get_logger().info(f"Current axis: {self.current_axis}\n \
                #                        Num saved: {len(self.saved_ft[self.current_axis])}")
                return
            
            if self.current_index < 3:
                self.current_axis = axis_dict[self.current_index]
                self.saved_ft[self.current_axis] = []
                self.saved_tf_ft[self.current_axis] = []
                target = aligned_joint_position(self.current_joint_states, self.current_axis)
                self.move_to_target_joint_pos(target)
                self.start_saving = True
            
            else:
                self.save_ft_raw()
                self.current_index = 0
                self.current_axis = axis_dict[self.current_index]
                self.get_logger().info("Initial pose")
                self.move_to_target_joint_pos(self.init_joint_states)
                self.run_gcomp = False
                
                try:
                    if os.path.exists(self.ft_saving_path) and os.path.exists(self.zero_ft_saving_path):
                        cgTool = ToolGravityCompensation(self.ft_saving_path, self.zero_ft_saving_path)
                        cg_result = cgTool.findCenterOfGravity(n=self.config['ft_num_measure']*3)
                        self.tool_cg = cg_result.x
                        with open(self.tool_cg_path, 'w') as f:
                            json.dump(self.tool_cg, f)
                except:
                    pass
                    

            # update mode
            # self.mode = mode
        
    def set_current_pose_as_init(self):
        self.init_joint_states = list(self.current_joint_states)
        if self.init_joint_states[1] == -1.57 and self.init_joint_states[3] == -1.57:
            for i in range(len(self.init_joint_states)):
                self.init_joint_states[i] += 0.1    # To avoid zero division error
        self.get_logger().info(f"Init joint states as current joint states: {self.init_joint_states}")
        
        
    def move_to_config_pose(self, config_pose, sleep_time=1.0):
        #assert config_pose in ['up', 'init', 'ready'], f"Invalid pose {config_pose}"
        # self.ur10_arm.set_start_state_to_current_state()
        # self.ur10_arm.set_goal_state(configuration_name=config_pose)
        
        # set plan start state to current state
        self.ur10_arm.set_start_state_to_current_state()
        
        if config_pose == 'init':
            # set joint group positions
            # https://moveit.picknik.ai/main/doc/api/python_api/_autosummary/moveit.core.robot_state.html
            self.robot_state.set_joint_group_positions(self.planning_group, self.init_joint_states)
            # set goal state to the initialized robot state
            self.ur10_arm.set_goal_state(robot_state=self.robot_state)
        else:
            raise ValueError(f"Invalid pose {config_pose}")
        
        # plan and execute
        plan_and_execute(self.ur10_moveit_py, self.ur10_arm, self.get_logger(), sleep_time=sleep_time)
        
    
    def move_to_target_joint_pos(self, target_joint_position, sleep_time=1.0):
        self.ur10_arm.set_start_state_to_current_state()
        self.get_logger().info(f"Target joint pos: {target_joint_position}")
        self.robot_state.set_joint_group_positions(self.planning_group, target_joint_position)
        self.ur10_arm.set_goal_state(robot_state=self.robot_state)
        plan_and_execute(self.ur10_moveit_py, self.ur10_arm, self.get_logger(), sleep_time=sleep_time)
    
        
    def change_to_velocity_controller(self):
        print("change to ", self.velocity_controller)
        res = self.switch_controller(self.base_controller, self.velocity_controller)
        return res
    
    def change_to_base_controller(self):
        print("change to ", self.base_controller)
        res = self.switch_controller(self.velocity_controller, self.base_controller)
        return res
    
    def switch_controller(self, deactivate_controllers, activate_controllers, strictness=2):
        """ 컨트롤러 변경 함수 """
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/switch_controller is not available!')
            return False

        req = SwitchController.Request()
        req.activate_controllers = [activate_controllers]
        req.deactivate_controllers = [deactivate_controllers]
        req.strictness = strictness  # 2 = STRICT, 1 = BEST_EFFORT

        # Duration 설정 (필수)
        req.timeout = Duration()
        req.timeout.sec = 2
        req.timeout.nanosec = 0

        future = self.switch_controller_client.call_async(req)
        # self.get_logger().info(f"future: {future.done()} {future.result()}")
        return True
        #rclpy.spin_until_future_complete(self, future)

        # if future.done() and future.result() is not None:
        #     if future.result().ok:
        #         self.get_logger().info(f'Successfully switched controllers: Activated {activate_controllers}, Deactivated {deactivate_controllers}')
        #         return True
        #     else:
        #         self.get_logger().error('Controller switch failed!')
        #         return False
        # else:
        #     self.get_logger().error('Service call timed out or failed.')
        #     return False
    
    def get_active_controllers(self):
        if not self.list_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/list_controllers is not available!')
            return []
        self.get_logger().info('Service /controller_manager/list_controllers is available!')
        
        req = ListControllers.Request()
        future = self.list_controller_client.call_async(req)
        self.get_logger().info(f"future: {future.done()} {future.result()}")
        # controllers = future.result().controller
        # active_controllers = [c.name for c in controllers if c.state == 'active']
        # self.get_logger().info(f'Active Controllers: {active_controllers}')
        return True

        # try:
        #     # 🟢 동기 서비스 호출 (10초 대기)
        #     future = self.list_controller_client.call(req)
        #     if future is not None:
        #         controllers = future.controller
        #         active_controllers = [c.name for c in controllers if c.state == 'active']
        #         self.get_logger().info(f'Active Controllers: {active_controllers}')
        #         return active_controllers
        #     else:
        #         self.get_logger().error('Received empty response from service!')
        #         return []
        # except Exception as e:
        #     self.get_logger().error(f'Service call failed: {str(e)}')
        #     return []

        # future = self.list_controller_client.call_async(ListControllers.Request())
        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     controllers = future.result().controller
        #     active_controllers = [c.name for c in controllers if c.state == 'active']
        #     self.get_logger().info(f'Active Controllers: {active_controllers}')
        #     return active_controllers
        # else:
        #     self.get_logger().error('Failed to get controller list!')
        #     return []

def main(args=None):
    # initialize ros2 node
    rclpy.init(args=args)
    
    # parse arguments
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='gazebo', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    # instantiate MoveItPy -> should be done outside of Node class!!
    ur10_moveit_py = MoveItPy(node_name="ur10_moveit_py")    
    
    # instantiate node
    node = ModeManager(args, ur10_moveit_py)
    
    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    