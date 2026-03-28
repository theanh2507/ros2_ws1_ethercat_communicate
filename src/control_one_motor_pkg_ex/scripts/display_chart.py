#!/usr/bin/env python3

import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class DisplayChart(Node):
        def __init__(self):
            super().__init__('display_chart_node')

            self.joint_names = []
            self.joint_positions = [0.0] * 6
            self.joint_velocities = [0.0] * 6
            self.joint_effort = [0.0] * 6

            self.RATED_CURRENT = 2.8            # dong dinh muc motor (A)

            self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10)

            
            self.pub_velocity = self.create_publisher(Float64MultiArray, "chart_velocity_topic", 10)
            self.pub_effort = self.create_publisher(Float64MultiArray, "chart_effort_topic", 10)
            self.create_timer(0.05, self.pub_convert_value)             # 20Hz

        def joint_state_callback(self, msg):
            if not self.joint_names:
                self.joint_names = msg.name
            
            for i in range(len(msg.name)):
                try:
                    name =  msg.name[i]
                    idx = int(name.split('_')[-1]) - 1

                    if 0 <= idx < 6:
                        self.joint_positions[idx] = msg.position[i]
                        self.joint_velocities[idx] = msg.velocity[i]
                        self.joint_effort[idx] = msg.effort[i]
                except (ValueError, IndexError):
                    continue

        def pub_convert_value(self):
            if not self.joint_names:
                return None

            joint_positions_convert = [0.0] * 6
            joint_velocities_convert = [0.0] * 6
            joint_effort_convert = [0.0] * 6

            for i in range (len(self.joint_names)):
                joint_positions_convert[i] = self.joint_positions[i]
                joint_velocities_convert[i] = (self.joint_velocities[i] / 10000) * 60
                joint_effort_convert[i] = (self.joint_effort[i] / 1000) * 2.8

            msg_v = Float64MultiArray()
            msg_v.data = joint_velocities_convert
            self.pub_velocity.publish(msg_v)

            msg_e = Float64MultiArray()
            msg_e.data = joint_effort_convert
            self.pub_effort.publish(msg_e)


def main():
    rclpy.init()
    node = DisplayChart()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()