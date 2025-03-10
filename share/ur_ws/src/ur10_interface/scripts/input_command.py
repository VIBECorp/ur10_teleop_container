#!/usr/bin/env python3
"""Joystick Agent."""
import pygame, sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

#####################################
# Change these to match your joystick
RIGHT_UP_AXIS = 4
RIGHT_SIDE_AXIS = 3
LEFT_UP_AXIS = 1
LEFT_SIDE_AXIS = 0
#####################################

class InputPublisher(Node):
    def __init__(self, fps=100, input_type='keyboard'):
        super().__init__('command_node')
        self.fps = fps
        self.publisher_ = self.create_publisher(Float64MultiArray, input_type+'_command', 10)
        timer_period = 1/fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        """Init."""
        self.input_type = input_type
        self.human_agent_action = np.zeros(6)
        self.button = np.zeros(1)
        
        if self.input_type == 'joystick':
            pygame.joystick.init()
            joysticks = [pygame.joystick.Joystick(x)
                        for x in range(pygame.joystick.get_count())]
            if len(joysticks) > 1:
                raise ValueError("There must be exactly 1 joystick connected.",
                                "Found ", len(joysticks))
            elif len(joysticks) == 0:
                raise ValueError("There is no joystick connected.")
            elif len(joysticks) == 1:   
                self.joy = joysticks[0]
                self.joy.init()
        elif self.input_type == 'keyboard':
            width, height = 640, 480
            screen = pygame.display.set_mode((width, height))
            
        pygame.init()
        
    def timer_callback(self):
        if self.input_type == 'joystick':  action, button = self._get_joystick_action()
        else:                              action, button = self._get_keyboard_action()  
        command = Float64MultiArray()
        command.data.append(action[0]) # x
        command.data.append(action[1]) # y
        command.data.append(action[2]) # z
        command.data.append(action[3]) # roll
        command.data.append(action[4]) # pitch
        command.data.append(action[5]) # yaw
        command.data.append(button) # button
        self.publisher_.publish(command)
    
    def _get_joystick_action(self):
        for event in pygame.event.get():
            # Joystick input
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == LEFT_SIDE_AXIS:
                    self.human_agent_action[1] = event.value
                elif event.axis == LEFT_UP_AXIS:
                    self.human_agent_action[0] = -1.0 * event.value
                if event.axis == RIGHT_SIDE_AXIS:
                    self.human_agent_action[5] = event.value
                elif event.axis == RIGHT_UP_AXIS:
                    self.human_agent_action[2] = -1.0 * event.value
            if event.type == pygame.JOYBUTTONDOWN:
                self.button[0] = event.button
                if self.button[0] == 1:
                    self.human_agent_action[3] = 1
                elif self.button[0] == 2:
                    self.human_agent_action[3] = -1
                if self.button[0] == 0:
                    self.human_agent_action[4] = 1
                elif self.button[0] == 3:
                    self.human_agent_action[4] = -1
            else: # button clear
                self.button[0] = -1
                self.human_agent_action[3] = self.human_agent_action[4] = 0
            
        return self.human_agent_action, self.button
    
    def _get_keyboard_action(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()

        keys = pygame.key.get_pressed()
        # Position axis 1 - lateral
        if    keys[pygame.K_a]: self.human_agent_action[0] = 1.0
        elif  keys[pygame.K_d]: self.human_agent_action[0] = -1.0
        else:                   self.human_agent_action[0] = 0.0
        # Position axis 2 - forward
        if    keys[pygame.K_s]: self.human_agent_action[1] = -1.0
        elif  keys[pygame.K_w]: self.human_agent_action[1] = 1.0
        else:                   self.human_agent_action[1] = 0.0
        # Position axis 3 - up
        if    keys[pygame.K_e]: self.human_agent_action[2] = 1.0
        elif  keys[pygame.K_c]: self.human_agent_action[2] = -1.0
        else:                   self.human_agent_action[2] = 0.0
        # Orientation axis 1 - roll
        if    keys[pygame.K_t]: self.human_agent_action[3] = 1.0
        elif  keys[pygame.K_g]: self.human_agent_action[3] = -1.0
        else:                   self.human_agent_action[3] = 0.0
        # Orientation axis 2 - pitch
        if    keys[pygame.K_r]: self.human_agent_action[4] = 1.0
        elif  keys[pygame.K_y]: self.human_agent_action[4] = -1.0
        else:                   self.human_agent_action[4] = 0.0
        # Orientation axis 3 - yaw
        if    keys[pygame.K_f]: self.human_agent_action[5] = 1.0
        elif  keys[pygame.K_h]: self.human_agent_action[5] = -1.0
        else:                   self.human_agent_action[5] = 0.0
        # Mode and speed
        if    keys[pygame.K_1]: self.button[0] = 6.0 # init mode
        elif  keys[pygame.K_2]: self.button[0] = 7.0 # teleop mode
        elif  keys[pygame.K_x]: self.button[0] = 5.0 # speed up
        elif  keys[pygame.K_z]: self.button[0] = 4.0 # speed down
        else:                   self.button[0] = -1.0
        return self.human_agent_action, self.button  


def main(args=None):
    rclpy.init(args=args)

    input_publisher = InputPublisher()
    rclpy.spin(input_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    input_publisher.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()