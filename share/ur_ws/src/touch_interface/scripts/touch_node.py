#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from std_msgs.msg import String, Float64MultiArray, Int32, Bool
import serial
import numpy as np

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


axis = {
    'Tx': 0,
    'Ty': 1,
    'Tz': 2,
    'Rx': 4,
    'Ry': 3,
    'Rz': 5,
    }

signs = {
    'Tx': 1,
    'Ty': 1,
    'Tz': 1,
    'Rx': 1,
    'Ry': -1,
    'Rz': 1,
}

axis_list = list(axis.keys())

class TouchCommandPublisher(Node):
    def __init__(self):
        super().__init__('touch_command_publisher')
        self.declare_parameter('port', '/root/share/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.current_mode = -1
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        
        self.touch_pose_pub = self.create_publisher(Float64MultiArray, 'touch_commands', 10)
        self.touch_btn_pub = self.create_publisher(String, 'touch_buttons', 10)
        self.mode_from_touch_pub = self.create_publisher(Int32, 'mode_from_touch', 10)
        
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Successfully connected to {port} at {baud} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            rclpy.shutdown()
            return
        
        # while rclpy.ok():
        #     try:
        #         self.current_mode = self.get_parameter('mode').get_parameter_value().integer_value
        #     except ParameterNotDeclaredException:
        #         self.get_logger().warn(f'Paramter Mode not declared yet.')
        #         rclpy.spin_once(self, timeout_sec=0.5)
        # self.send_mode(self.current_mode)
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Touch command publisher node has been started.')
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    def shutdown_callback(self, msg):
        if msg.data:
            rclpy.shutdown()
        
        
    def send_mode(self, mode=0):
        if mode == INIT:
            self.ser.write('INIT'.encode())
        elif mode == TELEOP:
            self.ser.write('TELE'.encode())
        elif mode == GCOMP:
            self.ser.write('GCOMP'.encode())
        elif mode == DIRECT_TRANS:
            self.ser.write('TRANS'.encode())
        elif mode == DIRECT_ROT:
            self.ser.write('ROT'.encode())
        self.get_logger().info('Mode sent to Touch')
            
            
    def mode_callback(self, msg):
        if self.current_mode != msg.data:
            self.send_mode(msg.data)
        self.current_mode = msg.data
        # self.send_mode(self.current_mode)
            
        
        
    def timer_callback(self):
        user_pose = np.zeros(6)
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received data: {data}')
            
            def code_in_try():
                values = data.split(',')
                if values[0] in axis_list:
                    user_pose[axis[values[0]]] = signs[values[0]] * float(values[1])
                    user_pose[:3] /= 1000.0  # Convert mm to m
                    user_pose[3:] = np.deg2rad(user_pose[3:])
                    msg = Float64MultiArray(data=user_pose)
                    self.touch_pose_pub.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
                    
                elif values[0] == 'btn':
                    button_msg = String(data=values[1])
                    self.touch_btn_pub.publish(button_msg)
                    self.get_logger().info(f'Published button: {button_msg.data}')
                    
                elif values[0] == 'TRANS':
                    msg = Int32()
                    msg.data = DIRECT_TRANS
                    self.mode_from_touch_pub.publish(msg=msg)
                    
                elif values[0] == 'ROT':
                    msg = Int32()
                    msg.data = DIRECT_ROT
                    self.mode_from_touch_pub.publish(msg=msg)
                
                elif values[0] == 'UTIL':
                    msg = Int32()
                    msg.data = REPLAY
                    self.mode_from_touch_pub.publish(msg=msg)
                    
                elif values[0] == 'INIT':
                    msg = Int32()
                    msg.data = INIT
                    self.mode_from_touch_pub.publish(msg=msg)
                    
                elif values[0] == 'TELE':
                    msg = Int32()
                    msg.data = TELEOP
                    self.mode_from_touch_pub.publish(msg=msg)
                
                    
            # code_in_try()
            try:
                code_in_try()
                # values = list(map(float, data.split(',')))
                # if values[0] in axis_list:
                #     user_pose[axis[values[0]]] = float(values[1])    
                #     msg = Float64MultiArray(data=user_pose)
                #     self.touch_pose_pub.publish(msg)
                #     self.get_logger().info(f'Published: {msg.data}')
                    
                # elif values[0] == 'btn':
                #     button_msg = String(data=values[1])
                #     self.touch_btn_pub.publish(button_msg)
                #     self.get_logger().info(f'Published button: {button_msg.data}')
                    
            except ValueError as e:
                self.get_logger().error(f'Error parsing data: {e}')
                
                
def main(args=None):
    rclpy.init(args=args)
    touch_command_publisher = TouchCommandPublisher()
    
    try:
        rclpy.spin(touch_command_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        touch_command_publisher.destroy_node()
        rclpy.shutdown()
        if touch_command_publisher.ser.is_open:
            touch_command_publisher.ser.close()
        touch_command_publisher.get_logger().info('Touch command publisher node has been shut down.')

if __name__ == '__main__':
    main()