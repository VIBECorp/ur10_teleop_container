#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Int32, Bool, String
from geometry_msgs.msg import Wrench, PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

from utils import get_package_dir, get_file_dir

import yaml
import json
import numpy as np
import os
import argparse
from ros_params import *

def load_parameters_from_config(args):
    # config 파일 로드
    config = None
    config_path = os.path.join(get_package_dir("ur10_interface"), 'config', f"config_{args.env}.yaml")
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    return config

class RecordAndPlay(Node):
    def __init__(self, args):
        super().__init__('record_and_play_node')
        self.config = load_parameters_from_config(args)
        
        for key, value in self.config.items():
            self.declare_parameter(key, value)
            
        self.saving_path = self.config['trajectory_saving_path']
        if not os.path.exists(self.saving_path):
            os.makedirs(self.saving_path)
        self.file_name = self.config['trajectory_saving_name']
        self.file_path = os.path.join(self.saving_path, self.file_name)
        self.planning_group = self.config['planning_group']
        self.current_pose = None
        self.current_joint_states = None
        self.mode = 1.0
        self.button = 0.0
        
        self.reset = True
        self.recorded_trajectory = {'type':[], 'task': [], 'joint': [], 'tool': [], 'time': []}
        self.start_continuous_recording = False
        self.start_time = 0
        self.continuous_trajectory = []
        

        self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        self.create_subscription(Float64MultiArray, 'keyboard_command', self.keyboard_command_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.create_subscription(PoseStamped, 'current_ee_pose', self.current_ee_pose_callback, 10)
        self.create_subscription(String, 'touch_buttons', self.touch_buttons_callback, 10)
        self.file_pub = self.create_publisher(String, 'trajectory_filename', 10)
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    def shutdown_callback(self, msg):
        if msg.data:
            self.reset = True
            self.start_continuous_recording = False
            self.start_time = 0
            rclpy.shutdown()
        
    
    def joint_state_callback(self, msg):
        temp = list(msg.position)  # 새로운 리스트로 변환하여 복사
        temp2 = temp[:]  # 깊은 복사 (Shallow Copy)

        # 인덱스 변경
        # temp2[0] = temp[5]
        # temp2[1] = temp[0]
        # temp2[2] = temp[1]
        # temp2[3] = temp[2]
        # temp2[4] = temp[3]
        # temp2[5] = temp[4]
        temp2[0] = temp[2]
        temp2[1] = temp[1]
        temp2[2] = temp[0]
        temp2[3] = temp[3]
        temp2[4] = temp[4]
        temp2[5] = temp[5]
        self.current_joint_states = temp2
        
        if self.start_continuous_recording:
            self.record_continuous()
            # self.get_logger().info(f"Recorded continuous point: {temp2}")
        
        
    def mode_callback(self, msg):
        self.mode = msg.data
        # print(self.mode)
        
        
    def touch_buttons_callback(self, msg):
        btn = msg.data
        
        if btn == 'save':
            self.record_discrete()
        elif btn == 'delete':
            self.remove_discrete()
        elif btn == 'write':
            self.save_trajectory()
        elif btn == 'start':
            self.get_logger().info("Start recording continuous trajectory")
            self.start_continuous_recording = True
            self.start_time = self.get_clock().now().nanoseconds
        elif btn == 'stop':
            self.start_continuous_recording = False
            self.start_time = 0
            self.get_logger().info("Stop recording continuous trajectory")
                
        
        
    
    def keyboard_command_callback(self, msg):
        temp = msg.data
        self.button = temp[-1]
        if self.button > 0 and self.reset:
            # print(self.button)
            self.reset = False
            if self.button == 100:  # i
                self.record_discrete()
            elif self.button == 101: # o
                self.remove_discrete()
            elif self.button == 99: # 0
                self.save_trajectory()
            elif self.button == 106:
                self.get_logger().info("Start recording continuous trajectory")
                self.start_continuous_recording = True
                self.start_time = self.get_clock().now().nanoseconds
            elif self.button == 107:
                self.start_continuous_recording = False
                self.start_time = 0
                self.get_logger().info("Stop recording continuous trajectory")
            
        elif self.button < 0 and self.reset is False:
            self.reset = True
        
    def current_ee_pose_callback(self, msg):
        self.current_pose = msg
        
    
    def save_trajectory(self):
        with open(self.file_path, 'w') as file:
            if len(self.recorded_trajectory['task']) > 0:
                json.dump(self.recorded_trajectory, file, indent=4)
            elif len(self.continuous_trajectory) > 0:
                json.dump(self.continuous_trajectory, file, indent=4)
            
        self.get_logger().info(f"Trajectory saved to {self.file_path}")
        self.file_pub.publish(String(data=self.file_path))
        
    def record_discrete(self):
        if self.mode in [TELEOP, DIRECT_TRANS, DIRECT_ROT]:
            task = [
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z,
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w,
                ]
            joint = [
                self.current_joint_states[0],
                self.current_joint_states[1],
                self.current_joint_states[2],
                self.current_joint_states[3],
                self.current_joint_states[4],
                self.current_joint_states[5]
                ]
            self.recorded_trajectory['type'].append(False)  # False for discrete
            self.recorded_trajectory['task'].append(task)
            self.recorded_trajectory['joint'].append(joint)
            self.recorded_trajectory['time'].append(0)
            self.recorded_trajectory['tool'].append(-1)
            self.get_logger().info(f"Recorded discrete point: {joint}")
    
    
    def remove_discrete(self):
        if self.mode in [TELEOP, DIRECT_TRANS, DIRECT_ROT]:
            xyz = [self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z]
            if len(self.recorded_trajectory['task']) != 0:
                idx = self.find_closest(xyz)
                if idx != -1:
                    for key in self.recorded_trajectory.keys():
                        self.recorded_trajectory[key].pop(idx)
                    self.get_logger().info(f"Removed discrete point at index {idx}")
        
    
    def find_closest(self, xyz):
        closest_idx = -1
        min_distance = float('inf')
        
        for i, point in enumerate(self.recorded_trajectory['task']):
            distance = np.linalg.norm(np.array(point[:3]) - np.array(xyz))
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        return closest_idx
    
    
    def record_continuous(self):
        if self.mode in [TELEOP, DIRECT_TRANS, DIRECT_ROT]:
            self.recorded_trajectory['time'].append(
                self.get_clock().now().nanoseconds - self.start_time
                )
            
            task = [
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z,
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w,
                ]
            joint = [
                self.current_joint_states[0],
                self.current_joint_states[1],
                self.current_joint_states[2],
                self.current_joint_states[3],
                self.current_joint_states[4],
                self.current_joint_states[5]
                ]
            self.recorded_trajectory['type'].append(True)  # True for continuous
            self.recorded_trajectory['task'].append(task)
            self.recorded_trajectory['joint'].append(joint)
            self.recorded_trajectory['tool'].append(-1)
        pass
    
    def play(self):
        pass
    
    
def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='real', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    # MoveItPy 초기화
    node = RecordAndPlay(args)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()