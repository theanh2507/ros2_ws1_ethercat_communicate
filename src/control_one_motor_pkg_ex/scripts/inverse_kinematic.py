#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class Kinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')
        
        self.joint_positions = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        self.joint_names = []

        self.PULSE_PER_REV = 10000.0  # 10000 xung
        self.LEAD_SCREW_MM = 5.0      # 5 mm
        self.PULSE_TO_MM = self.LEAD_SCREW_MM / self.PULSE_PER_REV
        self.L_HOME = 50.0            # mm

        self.RADIUS_BASE = 500      
        self.RADIUS_PLATFROM = 350

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.get_logger().info('Inverse Kinematic Node has been started and listening to /joint_states')

        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

        self.rB = 500.0  # mm
        self.rP = 350.0  # mm
        self.phiB = np.radians(15.0)
        self.phiP = np.radians(15.0)

        def get_coords_base(r, angles):
            return np.array([[r * np.cos(a), r * np.sin(a), 0] for a in angles])
        

        def get_coords_platfrom(r, angles):
            return np.array([[r * np.cos(a), r * np.sin(a), 1000] for a in angles])

        # base_angles = [
        #     self.phiB, -self.phiB, 
        #     np.radians(120) - self.phiB, np.radians(120) + self.phiB,
        #     np.radians(240) - self.phiB, np.radians(240) + self.phiB
        # ]

        base_angles = [
            self.phiB, np.radians(120) - self.phiB, 
            np.radians(120) + self.phiB, np.radians(240) - self.phiB,
            np.radians(240) + self.phiB, -self.phiB,
        ]

        self.B_coords = get_coords_base(self.rB, base_angles)
        self.get_logger().info(f'B_coords: {self.B_coords}')

        plat_angles = [
            np.radians(60) - self.phiP, np.radians(60) + self.phiP,
            np.radians(180) - self.phiP, np.radians(180) + self.phiP,
            np.radians(300) - self.phiP, np.radians(300) + self.phiP
        ]
        self.P_coords = get_coords_platfrom(self.rP, plat_angles)
        self.get_logger().info(f'P_coords: {self.P_coords}')

    def joint_state_callback(self, msg):
        if not self.joint_names:
            self.joint_names = msg.name

        for i in range(len(msg.name)):
            try:
                name = msg.name[i]
                idx = int(name.split('_')[-1]) - 1 
                
                if 0 <= idx < 6:
                    self.joint_positions[idx] = msg.position[i]
                    self.joint_velocities[idx] = msg.velocity[i]
            except (ValueError, IndexError):
                continue

        self.get_logger().info(f'Joint Position: {self.joint_positions}')
    
    def get_rotation_matrix(self, roll, pitch, yaw):
        # xoay quanh x
        Rx = np.array([[1, 0, 0],
                   [0, np.cos(np.radians(roll)), -np.sin(np.radians(roll))],
                   [0, np.sin(np.radians(roll)), np.cos(np.radians(roll))]])
        
        # xoay quanh y
        Ry = np.array([[np.cos(np.radians(pitch)), 0, np.sin(np.radians(pitch))],
                    [0, 1, 0],
                    [-np.sin(np.radians(pitch)), 0, np.cos(np.radians(pitch))]])
        
        # xoay quanh z
        Rz = np.array([[np.cos(np.radians(yaw)), -np.sin(np.radians(yaw)), 0],
                    [np.sin(np.radians(yaw)), np.cos(np.radians(yaw)), 0],
                    [0, 0, 1]])
        
        return Rx @ Ry @ Rz
    
    def calculate_inverse_kinematics(self, T, R):
        """
        T: Vector [x, y, z] - vi tri tam platfrom
        R: ma tran xoay 
        B_coords: toa do 6 diem khop duoi (he toa do B)
        P_coords: toa do 6 diem khop tren (he toa do P)
        """
        leg_lengths = []
        
        for i in range(6):     
            leg_vector = T + R @ self.P_coords[i] - self.B_coords[i]      # Li = T + R x Pi - Bi
            length = np.linalg.norm(leg_vector)                           # can cua x*x + y*y + z*z
            leg_lengths.append(length)
            
        return np.array(leg_lengths)
        
    def control_loop(self):
        target_T = np.array([0.0, 0.0, 300.0])
        target_R = self.get_rotation_matrix(0, 0, 0)

        self.get_logger().info(f'target_R: {target_R}')

        lengths = self.calculate_inverse_kinematics(target_T, target_R)
        self.get_logger().info(f'do dai cac chan la: {lengths}')

        if lengths is not None:
            self.publish_trajectory(lengths)
    
    def mm_to_pulse(self, target_position):
        self.target_pulse = (target_position - self.L_HOME) / self.PULSE_TO_MM
        return self.target_pulse

    def publish_trajectory(self, lengths):
        msg = JointTrajectory()
        msg.joint_names = [f'joint_{i+1}' for i in range(6)]
        
        point = JointTrajectoryPoint()
        
        point.positions = [float(self.mm_to_pulse(l)) for l in lengths]
        
        point.velocities = [0.0] * 6
        
        point.time_from_start = Duration(sec=2, nanosec=0) 

        msg.points.append(point)
    
        self.publisher_.publish(msg)
        
        # self.get_logger().info(f'Target Pulse Joint 1: {point.positions[0]}')

def main(args=None):
    rclpy.init(args=args)
    node = Kinematics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()