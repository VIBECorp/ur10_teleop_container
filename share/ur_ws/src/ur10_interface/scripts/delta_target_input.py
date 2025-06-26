#!/usr/bin/env python3
# -*- coding: utf8 -*- 
# from ur10_interface.common_utils import *
import argparse
import yaml
import os
import time
import numpy as np
import copy

from utils import get_package_dir, get_file_dir

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
AI = 4
MOVEIT = 5
IDLE = 6

mode_dict = {
    0: "INIT",
    1: "TELEOP",
    2: "TASK_CONTROL",
    3: "JOINT_CONTROL",
    4: "AI",
    5: "MOVEIT",
    6: "IDLE",
}

# ROS2 library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool

class DeltaTargetInput(Node):
    def __init__(self, args, fps=250):
        super().__init__('delta_target_input_node')
        
        # load parameters
        self.load_parameters_from_config(args)

        # initialize
        self.prefix = args.prefix
        self.env = self.config["env"]
        self.init_pose = self.config["init_pose"]
        self.init_joint_states = self.config["init_joint_states"]
        self.input_pos_gain = self.config["input_pos_gain"]
        self.input_ori_gain = self.config["input_ori_gain"]
        self.mode = self.config["mode"]

        # teleoperation variable
        self.pre_button = None
        self.joystick_command = [0.0] * 7
        self.joystick_command[6] = -1.0
        self.keyboard_command = [0.0] * 7
        self.keyboard_command[6] = -1.0
        self.ai_command = [0.0] * 7
        self.ai_command[6] = -0.1
        self.speed_level = 5  # 로봇 움직임 속도 - 1~10 단계

        # Publisher & Subscribers
        self.delta_target_input_pub = self.create_publisher(Float64MultiArray, "delta_target_input", 10)
        self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        self.create_subscription(Float64MultiArray, 'joystick_command', self.joystick_command_callback, 10)
        self.create_subscription(Float64MultiArray, 'keyboard_command', self.keyboard_command_callback, 10)
        self.create_subscription(Float64MultiArray, 'ai_command', self.ai_command_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / fps, self.loop)
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    def shutdown_callback(self, msg):
        if msg.data:            
            rclpy.shutdown()
        
    def load_parameters_from_config(self, args):
        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Upload parameters
        for key, value in self.config.items():
            self.declare_parameter(key, value)
            # self.get_logger().info(f"Loaded param: {key} = {value}")

    def input_conversion(self, command):
        x_input = command[1]
        y_input = command[0]
        z_input = command[2]
        roll_input = command[4]
        pitch_input = command[3]
        yaw_input = command[5]
        button = int(command[6])

        # change speed
        if button != self.pre_button:  # restrict the continuous change
            if button == 5:
                self.speed_level += 0.1
                if self.speed_level > 20:
                    self.speed_level = 20
            elif button == 4:
                self.speed_level -= 0.1
                if self.speed_level < 0:
                    self.speed_level = 0
            self.pre_button = button

        position_scale = self.input_pos_gain * self.speed_level
        orientation_scale = self.input_ori_gain * self.speed_level

        delta_target = [
            x_input * position_scale,
            y_input * position_scale,
            z_input * position_scale,
            roll_input * orientation_scale,
            pitch_input * orientation_scale,
            yaw_input * orientation_scale
        ]
        return delta_target

    def joystick_command_callback(self, msg):
        self.joystick_command = list(msg.data)

    def keyboard_command_callback(self, msg):
        self.keyboard_command = list(msg.data)

    def ai_command_callback(self, msg):
        self.ai_command = list(msg.data)

    def mode_callback(self, msg):
        self.mode = msg.data

    def loop(self):
        # access mode from parameter server
        mode = self.mode
        # self.get_logger().info(f"Current mode: {mode_dict[mode]}")
        delta_target_input = Float64MultiArray()
        if mode == TELEOP:
            # generate delta target input from joystick and keyboard command
            delta_target_joystick = self.input_conversion(self.joystick_command)
            delta_target_keyboard = self.input_conversion(self.keyboard_command)
            # combine joystick and keyboard input
            for i in range(6):
                delta_target_input.data.append(delta_target_joystick[i] + delta_target_keyboard[i]) 
        elif mode == AI:
            # generate delta target input from ai command
            delta_target_ai = self.input_conversion(self.ai_command)
            delta_target_input.data = delta_target_ai[:6]
        self.delta_target_input_pub.publish(delta_target_input)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='gazebo', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    # instantiate node
    node = DeltaTargetInput(args)
    
    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()