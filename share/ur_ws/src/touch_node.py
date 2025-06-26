#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import serial
import numpy as np

axis = {
    'Tx': 0,
    'Ty': 1,
    'Tz': 2,
    'Rx': 3,
    'Ry': 4,
    'Rz': 5,
    }
axis_list = list(axis.keys())

class TouchCommandPublisher(Node):
    def __init__(self):
        super().__init__('touch_command_publisher')
        self.declare_parameter('port', '/root/share/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.touch_pose_pub = self.create_publisher(Float64MultiArray, 'touch_commands', 10)
        self.touch_btn_pub = self.create_publisher(String, 'touch_buttons', 10)
        
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Successfully connected to {port} at {baud} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            rclpy.shutdown()
            return
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Touch command publisher node has been started.')
        
        
    def timer_callback(self):
        user_pose = np.zeros(6)
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received data: {data}')
            
            def code_in_try():
                values = data.split(',')
                if values[0] in axis_list:
                    user_pose[axis[values[0]]] = float(values[1])
                    user_pose[:3] /= 1000.0  # Convert mm to m
                    user_pose[3:] = np.deg2rad(user_pose[3:])
                    msg = Float64MultiArray(data=user_pose)
                    self.touch_pose_pub.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
                    
                elif values[0] == 'btn':
                    button_msg = String(data=values[1])
                    self.touch_btn_pub.publish(button_msg)
                    self.get_logger().info(f'Published button: {button_msg.data}')
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