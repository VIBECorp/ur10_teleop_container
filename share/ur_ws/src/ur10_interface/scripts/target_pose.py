#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# from ur10_interface.common_utils import *
import argparse
import yaml
import os
import time
import numpy as np
import copy

from ament_index_python.packages import get_package_share_directory

def get_package_dir(package_name):
    share_dir = get_package_share_directory(package_name)
    package_dir = share_dir.replace('install', 'src').removesuffix(f'/share/{package_name}')
    return package_dir

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
    8: "DIRECT"
}


import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped, Wrench
from std_msgs.msg import Float64MultiArray, Int32, Bool
from std_srvs.srv import Trigger
from tutorial_interfaces.srv import SetTargetPose
from tf_transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion


class TargetPose(Node):
    def __init__(self, args):
        super().__init__('target_pose_node')
        
        # Load parameters
        self.load_parameters_from_config(args)

        # Initialize
        self.prefix = args.prefix
        self.env = self.config["env"]
        self.init_pose = self.config["init_pose"]
        self.mode = self.config["mode"]
        self.current_mode = self.mode
        self.target_pose = copy.deepcopy(self.init_pose)
        self.last_success_target_pose = copy.deepcopy(self.init_pose)
        self.delta_target_input = [0.0] * 6
        self.ik_success = False
        self.target_pose_msg = self.update_target_pose()
        self.current_pose = None
        self.current_tf_ft = None
        self.force_th = self.config['force_thresh']
        self.force_pose_gain = self.config['force_pose_gain']
        
        # Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        self.target_pose_pub = self.create_publisher(PoseStamped, "target_pose", 10)
        self.create_subscription(Float64MultiArray, 'delta_target_input', self.delta_target_input_callback, 10)
        self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        self.create_subscription(Bool, 'ik_success', self.ik_success_callback, 10)
        self.create_subscription(PoseStamped, 'current_ee_pose', self.current_pose_callback, 10)
        self.create_subscription(Wrench, 'compensated_ft_wrt_base', self.comp_ft_callback, 10)
        
        # Services & Clients
        self.create_service(Trigger, 'reset_target_pose', self.reset_target_pose)
        self.create_service(SetTargetPose, 'set_target_pose', self.set_target_pose)

        # Timer
        self.timer = self.create_timer(1.0 / 250, self.loop)
        
    
    def comp_ft_callback(self, msg):
        temp = [msg.force.x,
                msg.force.y,
                msg.force.z,
                msg.torque.x,
                msg.torque.y,
                msg.torque.z]
        self.current_tf_ft = np.array(temp[:])
        
        
    def direct_teaching_pose_update(self):
        delta_target = np.zeros(6)
        
        if self.current_tf_ft is None:
            pass
        else:
            idx = np.where(np.abs(self.current_tf_ft[:3]) >= self.force_th)[0]
            if len(idx) > 0:
                delta_target[idx] = (np.sign(self.current_tf_ft)[idx] * np.abs(self.current_tf_ft)[idx] - self.force_th) * self.force_pose_gain
                print(delta_target)
            
                for i in range(len(self.target_pose)):
                    self.target_pose[i] += delta_target[i]
                # print(self.target_pose)
            else:
                self.set_target_as_current(self.current_pose)
        
    
    def current_pose_callback(self, msg):
        self.current_pose = msg
        
        
    def loop(self):
        if self.current_pose is None:
            self.get_logger().info("Current pose is None")
            return
        # self.get_logger().info(f"Current pose: {self.current_pose}")
        
        mode = self.mode
        if mode in [INIT, TELEOP, AI, DIRECT]:
            # self.get_logger().info(f"Env: {self.env}, Mode: {mode_dict[mode]}")
            if mode == INIT:
                self.set_target_as_current(self.current_pose)
                target_pose_msg = self.current_pose
                if self.current_mode != mode: self.get_logger().info("Target pose becomes current pose")
                
                # self.target_pose = copy.deepcopy(self.init_pose)
                # # self.get_logger().info(f"Init pose: {self.target_pose}")
                # if self.current_mode != mode: self.get_logger().info("Target pose initialized")
                
                # target_pose_msg = self.convert_to_pose_msg(self.target_pose)
                
            
            elif mode in [TELEOP, AI]:
                self.update_target_pose() # add delta to target pose
                if self.current_mode != mode: self.get_logger().info("Target pose is updating by delta input")
                target_pose_msg = self.convert_to_pose_msg(self.target_pose)
                
                
            elif mode is DIRECT:
                self.direct_teaching_pose_update()
                target_pose_msg = self.convert_to_pose_msg(self.target_pose)

            
            if not target_pose_msg is None:
                self.target_pose_pub.publish(target_pose_msg)
        
        self.current_mode = mode
        
    def load_parameters_from_config(self, args):
        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Upload parameters
        for key, value in self.config.items():
            self.declare_parameter(key, value)
            # self.get_logger().info(f"Loaded param: {key} = {value}")

    def update_target_pose(self):
        # exception handling
        if len(self.delta_target_input) == 0:
            self.delta_target_input = [0.0] * 6
            
        # add delta to target pose
        # if not self.ik_success:
        #     self.target_pose = copy.deepcopy(self.last_success_target_pose)         
        for i in range(6):
            self.target_pose[i] += self.delta_target_input[i]

    def convert_to_pose_msg(self, target_pose):
        # Quaternion 변환
        q_new = quaternion_from_euler(self.target_pose[3], self.target_pose[4], self.target_pose[5])
        target_orientation = Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])

        # PoseStamped 메시지 생성
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x = self.target_pose[0]
        ps.pose.position.y = self.target_pose[1]
        ps.pose.position.z = self.target_pose[2]
        ps.pose.orientation = target_orientation

        return ps
    
    def set_target_as_current(self, pose_stamped: PoseStamped):
        # 위치 (Translation)
        translation = (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        )

        # Quaternion -> Euler 변환
        quaternion = (
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)  # (roll, pitch, yaw) 반환
        
        self.target_pose[0] = translation[0]
        self.target_pose[1] = translation[1]
        self.target_pose[2] = translation[2]
        self.target_pose[3] = euler[0]
        self.target_pose[4] = euler[1]
        self.target_pose[5] = euler[2]
    

    def reset_target_pose(self, request, response):
        # self.target_pose = copy.deepcopy(self.init_pose)
        self.set_target_as_current(self.current_pose)
        response.success = True
        response.message = "Target pose reset"
        return response

    def set_target_pose(self, request, response):
        self.target_pose = list(request.target_pose.data)
        response.success = True
        return response

    def delta_target_input_callback(self, msg):
        self.delta_target_input = msg.data.tolist()
            
    def mode_callback(self, msg):
        self.mode = msg.data
        
    def ik_success_callback(self, msg):
        self.ik_success = msg.data


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='real', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    # instantiate node
    node = TargetPose(args)
    
    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()