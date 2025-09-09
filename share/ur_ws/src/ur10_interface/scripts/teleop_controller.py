#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import argparse
import yaml
import os
import numpy as np
import time

from utils import get_package_dir, get_file_dir

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
AI = 4
MOVEIT = 5
IDLE = 6
GCOMP = 7
DIRECT_TRANS = 8
DIRECT_ROT = 9
REPLAY = 10

mode_dict = {
    0: "INIT",
    1: "TELEOP",
    2: "TASK_CONTROL",
    3: "JOINT_CONTROL",
    4: "AI",
    5: "MOVEIT",
    6: "IDLE",
    7: "GCOMP",
    8: "DIRECT_TRANS",
    9: "DIRECT_ROT",
    10: "REPLAY",
    -1: "READY",
}

## standard library
import numpy as np

## ros library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool
from sensor_msgs.msg import JointState

## Function Definition
def Duration2Time(duration):
    return duration.sec + duration.nanosec * 1e-9


class TeleopController(Node):
    def __init__(self, args):
        super().__init__('teleop_controller_node')
        
        # load parameters
        self.load_parameters_from_config(args)

        # initialize
        self.prefix = args.prefix
        self.env = self.config["env"]
        self.init_pose = self.config["init_pose"]
        self.mode = self.config["mode"]
        self.current_mode = self.mode
        self.p_gain = self.config["task_p_gain"]
        self.d_gain = self.config["task_d_gain"]
        self.velocity_controller = self.config["velocity_controller"]

        self.target_joints = np.zeros(6)
        self.current_joints = np.zeros(6)
        self.pre_joint_errors = np.zeros(6)
        self.joint_vel_msg = Float64MultiArray()
        
        self.joint_manual_control = False
        self.manual_reset_int = 1.0
        self.last_time_manual_input = time.time()

        # Subscriber 설정
        self.target_joint_sub = self.create_subscription(
            Float64MultiArray, 
            '/ik_result', 
            self.target_joint_callback, 
            10)

        self.current_joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.current_joint_callback, 
            10)
        
        self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        self.create_subscription(Float64MultiArray, 'delta_target_joint_input', self.delta_target_joint_input_callback, 10)

        # Publisher 설정
        self.vel_pub = self.create_publisher(Float64MultiArray, f'/{self.velocity_controller}/commands', 10)

        # 타이머를 사용하여 주기적으로 control_loop 실행
        self.timer = self.create_timer(1.0 / 250.0, self.control_loop)  # 250Hz 실행
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    
    def delta_target_joint_input_callback(self, msg):
        self.joint_manual_control = True
        delta = np.array(msg.data.tolist())
        target_joint_position = self.current_joints.copy()
        
        if (len(delta) == len(self.current_joints)):
            target_joint_position += delta
        
        if self.mode in [TELEOP, TASK_CONTROL, JOINT_CONTROL, IDLE, AI, DIRECT_TRANS, DIRECT_ROT]:
            self.target_joints = target_joint_position  
            self.last_time_manual_input = time.time()
        
    
        
    def shutdown_callback(self, msg):
        if msg.data:
            self.joint_vel_msg.data = np.zeros(6)    
            self.vel_pub.publish(self.joint_vel_msg)    
            rclpy.shutdown()
        
    def mode_callback(self, msg):
        self.mode = msg.data

    def control_loop(self):
        # self.joint_vel_msg.data = np.zeros(6)
        # if self.mode == TELEOP or self.mode == DIRECT:
        #     try:
        #         # Compute control input
        #         joint_errors = np.array(self.target_joints) - np.array(self.current_joints)
        #         self.joint_vel_msg.data = self.p_gain * joint_errors + self.d_gain * (joint_errors - self.pre_joint_errors)
        #         self.pre_joint_errors = joint_errors
        #     except Exception as e:
        #         self.joint_vel_msg.data = np.zeros(6)
                
        # self.vel_pub.publish(self.joint_vel_msg)
        self.joint_vel_msg.data = np.zeros(6)

        try:
            # Compute control input
            joint_errors = np.array(self.target_joints) - np.array(self.current_joints)
            self.joint_vel_msg.data = self.p_gain * joint_errors + self.d_gain * (joint_errors - self.pre_joint_errors)
            # vel_command = self.p_gain * joint_errors + self.d_gain * (joint_errors - self.pre_joint_errors)
            # idx = np.where(vel_command < 0.000001)
            # vel_command[idx] = 0.0
            # self.joint_vel_msg.data = vel_command
            self.pre_joint_errors = joint_errors
            if time.time() - self.last_time_manual_input > self.manual_reset_int:
                self.joint_manual_control = False
        except Exception as e:
            self.joint_vel_msg.data = np.zeros(6)
                
        self.vel_pub.publish(self.joint_vel_msg)


    def stop(self):
        self.joint_vel_msg.data = np.zeros(6)
        self.vel_pub.publish(self.joint_vel_msg)

    def target_joint_callback(self, msg):
        if not self.joint_manual_control:
            self.target_joints = msg.data

    def current_joint_callback(self, msg):  
        current_joints = list(msg.position)
        if self.env == 'real':
            # real hardware에서 나온 joint states 순서 변경
            # converted_joints = [
            #     current_joints[5], 
            #     current_joints[0], 
            #     current_joints[1], 
            #     current_joints[2], 
            #     current_joints[3], 
            #     current_joints[4]
            #     ]
            converted_joints = [
                current_joints[2], 
                current_joints[1], 
                current_joints[0], 
                current_joints[3], 
                current_joints[4], 
                current_joints[5]
                ]
            current_joints = converted_joints
        self.current_joints = current_joints
        
    def load_parameters_from_config(self, args):
        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Upload parameters
        for key, value in self.config.items():
            self.declare_parameter(key, value)
            # self.get_logger().info(f"Loaded param: {key} = {value}")


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='gazebo', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    # instantiate node
    node = TeleopController(args)
    
    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()