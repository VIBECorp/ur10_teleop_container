#!/usr/bin/env python3

import os
import numpy as np
import json
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench, PoseStamped
from mode_manager import get_file_dir, get_package_dir

class DirectTeachingControl(Node):
    def __init__(self, args):
        super().__init__('direct_teaching_node')
        self.load_parameters_from_config(args)
        
        self.create_subscription(Wrench, 'compensated_ft_wrt_base', self.comp_ft_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        
        
        
    def load_parameters_from_config(self, args):
        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Upload parameters
        for key, value in self.config.items():
            self.declare_parameter(key, value)
            # self.get_logger().info(f"Loaded param: {key} = {value}")