#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool
import yaml
import os
import argparse
from utils import get_package_dir, get_file_dir

def transform_to_pose(transform: TransformStamped) -> PoseStamped:
    pose = PoseStamped()
    
    # 헤더 정보 복사
    pose.header = transform.header
    
    # 위치 정보 (translation)
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z

    # 회전 정보 (quaternion)
    pose.pose.orientation = transform.transform.rotation
    
    return pose

class TFBroadcaster(Node):
    def __init__(self, args):
        super().__init__('ee_tf_broadcaster')
        self.load_parameters_from_config(args)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pose_publisher = self.create_publisher(PoseStamped, 'current_ee_pose', 10)
        
        # 업데이트 속도를 100Hz로 설정 (0.01초마다 실행)
        self.timer = self.create_timer(0.01, self.publish_pose)
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    def shutdown_callback(self, msg):
        if msg.data:            
            rclpy.shutdown()
        
    def get_pose(self, from_frame: str, to_frame: str) -> PoseStamped:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            # print("Trnasform found")
            return transform_to_pose(transform)
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    def publish_pose(self):
        pose = self.get_pose(self.config['base_link'], self.config['end_effector_link'])
        if pose is not None:
            self.pose_publisher.publish(pose)
        
        
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
    rclpy.init()
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='real', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    node = TFBroadcaster(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
