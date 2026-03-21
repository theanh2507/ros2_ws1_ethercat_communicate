#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ethercat_msgs.srv import SetSdo, GetSdo
import time
import sensor_msgs.msg import JointState 

class DisplayStatusMotor(Node):
    def __init__(self):
        super().__init__('display_status_motor')
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        
        pass