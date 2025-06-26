#!/usr/bin/env python3

import os
import argparse
import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool
from geometry_msgs.msg import Wrench, PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import yaml

from utils import get_package_dir, get_file_dir

labels = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']
colors = ['r', 'g', 'b', 'k', 'c', 'm']

class ToolGravityCompensation:
    def __init__(self, tool_path, zero_path):
        self.path_raw = tool_path
        self.path_zero = zero_path
        
        self.zeros = json.load(open(self.path_zero, 'r'))
        for key in self.zeros.keys():
            self.zeros[key] = np.array(self.zeros[key])
        self.ft0 = np.mean(self.zeros['z'], axis=0)
        
        self.raw = json.load(open(self.path_raw, 'r'))
        for key in self.raw.keys():
            self.raw[key] = np.array(self.raw[key])
        
    def drawGraph(self):
        plt.figure(figsize=(15,5))
        for idx, axis in enumerate(self.raw.keys()):
            plt.subplot(1,3,idx+1)
            for i in range(self.raw[axis].shape[1]):
                plt.plot(self.raw[axis][:,i], colors[i]+'.--', label=labels[i])
                
            mean = np.mean(self.raw[axis], axis=0)
            norm = np.linalg.norm(mean[:3])
            print("Mean:", mean, "Norm:", norm)
            plt.grid()
            plt.title(f'{axis}-axis aligned.')
            plt.xlabel('Samples')
            plt.ylabel('Data')
            plt.legend()
        plt.tight_layout()
        plt.show()
        
        
        axis = 'z'
        plt.figure(figsize=(5,5))
        for i in range(self.zeros[axis].shape[1]):
            plt.plot(self.zeros[axis][:,i], colors[i]+'.--', label=labels[i])
        plt.grid()
        plt.title(f'{axis}-axis aligned')
        plt.xlabel('Samples')
        plt.ylabel('Data')
        plt.legend()
        plt.tight_layout()
        plt.show()
        
        
    def findCenterOfGravity(self, axis='all', num_samples=1000000):
        if axis == 'all':
            entire = None
            for key in self.raw.keys():
                if entire is None:
                    entire = self.raw[key]
                else:
                    entire = np.concatenate([entire, self.raw[key]], axis=0)
            np.random.shuffle(entire)
            
            if num_samples >= len(entire):
                samples = entire
            else:
                samples = entire[:num_samples]
        
        else:
            temp = self.raw[axis]
            np.random.shuffle(temp)
            
            if num_samples >= len(temp):
                samples = temp
            else:
                samples = temp[:num_samples]
        
        F = np.zeros((3 * num_samples, 6))
        t = np.zeros(3 * num_samples)
        
        for idx, ft in enumerate(samples):
            fx, fy, fz, tx, ty, tz = ft
            t[idx*3+0] = tx
            t[idx*3+1] = ty
            t[idx*3+2] = tz
            
            F[idx*3+0] = np.array([0, -fz, fy, 1, 0, 0])
            F[idx*3+1] = np.array([fz, 0, -fx, 0, 1, 0])
            F[idx*3+2] = np.array([-fy, fx, 0, 0, 0, 1])
        
        def func(x, F, t):
            return np.dot(F, x) - t
        
        p_init = np.array([0, 0, 0, 0, 0, 0])
        
        res = least_squares(func, p_init, args=(F, t))
        
        return res


class ToolGravityCompensationNode(Node):
    def __init__(self, args):
        super().__init__('tool_gravity_compensation_node')
        
        self.load_parameters_from_config(args)
        self.ft_saving_path = get_file_dir('ur10_interface', self.config['ft_raw_save_to'])
        self.tf_ft_saving_path = get_file_dir('ur10_interface', self.config['tf_ft_save_to'])
        self.force_thresh_offset = self.config['force_thresh_offset']
        self.torque_thresh_offset = self.config['torque_thresh_offset']
        self.FT_facing_z = None
        
        self.tool_cg_path = get_file_dir('ur10_interface', self.config['tool_cg_saved'])
        self.tool_cg = None
        self.compensated_ft = [0] * 6
        self.current_pose = None
        self.current_ft_data = None
        self.rotated_ft_data = [0] * 6
        
        self.update_th = False
        self.measure_for_th = []
        self.ft_thresh = [0.0] * 6
        
        self.create_subscription(Wrench, self.config['ft_sensor_node'], self.ft_data_callback, 10)
        self.create_subscription(PoseStamped, 'current_ee_pose', self.current_pose_callback, 10)
        self.create_subscription(Bool, 'update_ft_thresh', self.update_ft_thresh, 10)
        
        self.comp_ft_wrench_pub = self.create_publisher(Wrench, 'compensated_ft', 10)
        self.tf_comp_pub = self.create_publisher(Wrench, 'compensated_ft_wrt_base', 10)
        self.comp_norm_pub = self.create_publisher(Float64MultiArray, 'compensated_norm_ft', 10)
        self.ft_thresh_pub = self.create_publisher(Float64MultiArray, 'ft_thresh', 10)
        self.update_ft_thresh_pub = self.create_publisher(Bool, 'update_ft_thresh', 10)
        
        self.timer = self.create_timer(1.0, self.get_latest)
        self.th_timer = self.create_timer(0.1, self.pub_ft_thresh)
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    def shutdown_callback(self, msg):
        if msg.data:
            rclpy.shutdown()
        
        
    def update_ft_thresh(self, msg):
        self.update_th = msg.data
        
        
    def get_latest(self):
        try:
            if os.path.exists(self.tool_cg_path):
                self.tool_cg = json.load(open(self.tool_cg_path, 'r'))
            else:
                self.tool_cg = [0] * 6
                
            if os.path.exists(self.tf_ft_saving_path):
                temp = json.load(open(self.tf_ft_saving_path, 'r'))
                self.FT_facing_z = np.mean(np.array(temp[self.config['set_axis']]), axis=0)
            else:
                self.FT_facing_z = [0] * 6
                
        except:
            pass
    
        
    def current_pose_callback(self, msg):
        self.current_pose = msg
        
    def ft_data_callback(self, msg):
        self.current_ft_data = msg
        
        if self.FT_facing_z is None or self.current_pose is None:
            pass
        else:
            qx = self.current_pose.pose.orientation.x
            qy = self.current_pose.pose.orientation.y
            qz = self.current_pose.pose.orientation.z
            qw = self.current_pose.pose.orientation.w
            
            quat = [qx, qy, qz, qw]
            
            euler = euler_from_quaternion(quat)  # (roll, pitch, yaw) 반환
            
            rot = R.from_euler('xyz', euler).as_matrix()  # 'xyz' 순서로 회전 행렬 생성
            
            F = self.FT_facing_z[:3]
            T = self.FT_facing_z[3:]
            
            rF = rot.T @ F
            rT = rot.T @ T
            
            self.rotated_ft_data[0] = rF[0]
            self.rotated_ft_data[1] = rF[1]
            self.rotated_ft_data[2] = rF[2]
            self.rotated_ft_data[3] = rT[0]
            self.rotated_ft_data[4] = rT[1]
            self.rotated_ft_data[5] = rT[2]
            
            self.compensated_ft[0] = self.current_ft_data.force.x - self.rotated_ft_data[0]
            self.compensated_ft[1] = self.current_ft_data.force.y - self.rotated_ft_data[1]
            self.compensated_ft[2] = self.current_ft_data.force.z - self.rotated_ft_data[2]
            self.compensated_ft[3] = self.current_ft_data.torque.x - self.rotated_ft_data[3]
            self.compensated_ft[4] = self.current_ft_data.torque.y - self.rotated_ft_data[4]
            self.compensated_ft[5] = self.current_ft_data.torque.z - self.rotated_ft_data[5]
            
            comp_msg = Wrench()
            comp_msg.force.x = self.compensated_ft[0]
            comp_msg.force.y = self.compensated_ft[1]
            comp_msg.force.z = self.compensated_ft[2]
            comp_msg.torque.x = self.compensated_ft[3]
            comp_msg.torque.y = self.compensated_ft[4]
            comp_msg.torque.z = self.compensated_ft[5]
            
            self.comp_ft_wrench_pub.publish(msg=comp_msg)
            
            comp_f_norm = np.linalg.norm(np.array(self.compensated_ft[:3]))
            comp_t_norm = np.linalg.norm(np.array(self.compensated_ft[3:]))
            ft_array = Float64MultiArray()
            ft_array.data = np.array([comp_f_norm, comp_t_norm])
            self.comp_norm_pub.publish(ft_array)
            
            rFb = rot @ self.compensated_ft[:3]
            rTb = rot @ self.compensated_ft[3:]
            
            tf_comp_msg = Wrench()
            tf_comp_msg.force.x = rFb[0]
            tf_comp_msg.force.y = rFb[1]
            tf_comp_msg.force.z = rFb[2]
            tf_comp_msg.torque.x = rTb[0]
            tf_comp_msg.torque.y = rTb[1]
            tf_comp_msg.torque.z = rTb[2]
            
            self.tf_comp_pub.publish(msg=tf_comp_msg)
            
            if self.update_th:
                if len(self.measure_for_th) < self.config['thresh_num_measure']:
                    self.measure_for_th.append([rFb[0], rFb[1], rFb[2], rTb[0], rTb[1], rTb[2]])
                else:
                    self.update_th = False
                    self.cal_ft_threshold()
                    
                    self.measure_for_th = []
                    msg = Bool()
                    msg.data = self.update_th
                    self.update_ft_thresh_pub.publish(msg=msg)
                    
                    
    
    def cal_ft_threshold(self):
        samples = np.array(self.measure_for_th)
        ft_min = np.min(samples, axis=0)
        ft_max = np.max(samples, axis=0)
        
        
        for idx, (Min, Max) in enumerate(zip(ft_min, ft_max)):
            if np.abs(Min) < np.abs(Max):
                self.ft_thresh[idx] = np.abs(Max)
            else:
                self.ft_thresh[idx] = np.abs(Min)
        self.ft_thresh = np.array(self.ft_thresh)
        self.ft_thresh[:3] *= (1.0 + self.force_thresh_offset)
        self.ft_thresh[3:] *= (1.0 + self.torque_thresh_offset)
    
    
    def pub_ft_thresh(self):
        msg = Float64MultiArray()
        msg.data = self.ft_thresh
        self.ft_thresh_pub.publish(msg=msg)
        

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
    parser.add_argument("--env", type=str, default='real', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    node = ToolGravityCompensationNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()



# if __name__ == '__main__':
#     tool_path = 'src/ur10_interface/tool_gravity_ft_raw_tf.json'
#     zero_path = 'src/ur10_interface/zero_ft_raw.json'
#     tgc = ToolGravityCompensation(tool_path, zero_path)
    
#     tgc.drawGraph()
#     # n=30000
#     # z = tgc.findCenterOfGravity('z', n)
#     # x = tgc.findCenterOfGravity('x', n)
#     # y = tgc.findCenterOfGravity('y', n)
#     # all = tgc.findCenterOfGravity(num_samples=n)
    

#     # print(z.x)
#     # print("------------------------------")
#     # print(x.x)
#     # print("------------------------------")
#     # print(y.x)
#     # print("------------------------------")
#     # print(all.x)