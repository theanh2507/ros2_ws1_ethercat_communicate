#!/usr/bin/env python3

import time
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

        self.L_HOME = 30.0            # mm
        self.L_LEG = 1012.0           # 930 + 54
        self.Z_OFFSET = 200           # mm

        self.circle_points = []
        self.current_point_idx = 0
        self.is_running_circle = False


        self.RADIUS_BASE = 500      
        self.RADIUS_PLATFROM = 350

        self.TIME_TRAJECTORY = 0

        self.received_first_state = False

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.get_logger().info('Inverse Kinematic Node has been started and listening to /joint_states')

        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/trajectory_controller/joint_trajectory', 
            10)
        
        self.timer = None
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.rB = 500.0  # mm
        self.rP = 350.0  # mm
        self.phiB = np.radians(15.0)
        self.phiP = np.radians(15.0)

        def get_coords_base(r, angles):
            return np.array([[r * np.cos(a), r * np.sin(a), 0] for a in angles])
        

        def get_coords_platfrom(r, angles):
            return np.array([[r * np.cos(a), r * np.sin(a), 982] for a in angles])


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


        ########################

        # self.trajectory_circle(radius=150.0, target_velocity=30.0, points = 200.0)
        # self.timer = None

        # self.init_timer = self.create_timer(0.5, self.wait_for_motors_and_start)

    def wait_for_motors_and_start(self):
        # kiem cho cho nhan duoc du lieu cua joint state
        if not hasattr(self, 'received_first_state') or not self.received_first_state:
            self.get_logger().warn('Dang doi du lieu tu motor (JointState)...', throttle_duration_sec=2.0)
            return

        self.init_timer.cancel()
        self.get_logger().info('Da nhan du lieu motor. Bat dau tinh toan quy dao...')
        self.trajectory_circle(radius=100.0, target_velocity=30.0, points=300.0)

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
                    self.received_first_state = True
            except (ValueError, IndexError):
                continue
        self.received_first_state = True

        # self.get_logger().info(f'Joint Position: {self.joint_positions}')
    
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
        
        return Rz @ Ry @ Rx
    
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

        target_T = np.array([0.0, 0.0, self.Z_OFFSET + 0.0])
        target_R = self.get_rotation_matrix(0, 0.0, 0)
        self.get_logger().info(f'Ma tran xoay la: {target_R}')

        lengths_mm = self.calculate_inverse_kinematics(target_T, target_R)
        self.get_logger().info(f'Gia tri do dai xy lanh tinh tu 2 mp base va plat: {[round(float(l), 4) for l in lengths_mm]}')
        
        if lengths_mm is not None:
            lengths_mm= [
                (l - self.L_HOME - self.L_LEG) for l in lengths_mm]
            
            formatted_lengths = [f"{l:.4f}" for l in lengths_mm]
            self.get_logger().info(f'Gia tri do dai xy lanh can di chuyen tinh tu home la: {formatted_lengths}')
        
            real_cylinder_lengths = [f"{l + self.L_HOME :.4f}" for l in lengths_mm]

            self.get_logger().info(f'Gia tri do dai xy lanh 6 chan: {real_cylinder_lengths}')

            self.publish_trajectory(lengths_mm)

            self.timer.cancel()

    
    def mm_to_pulse(self, target_position):
        self.target_pulse = (-target_position) / self.PULSE_TO_MM
        return self.target_pulse

    def publish_trajectory(self, lengths):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [f'joint_{i+1}' for i in range(6)]
        
        point = JointTrajectoryPoint()
        
        safe_lengths = [np.clip(l, -20.0, 370.0) for l in lengths]

        point.positions = [round(float(self.mm_to_pulse(l)), 4) for l in safe_lengths]
        
        point.velocities = [0.0] * 6
        
        point.time_from_start = Duration(sec=5, nanosec=0) 

        msg.points.append(point)
    
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Target Pulse 6 Joint : {point.positions}')


    def generate_trajectory(self, radius, target_velocity, points_count):
        duration = (2 * math.pi * radius) / target_velocity
        self.get_logger().info(f'thoi gian hoan thanh 1 quy dao la: {duration}')

        if duration < 5.0: duration = 5.0 
        
        trajectory = []
        dt = duration / points_count

        for i in range(int(points_count)):
            t = i * dt
            theta = (2 * math.pi * t) / duration
            
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            z = self.Z_OFFSET
            
            trajectory.append({
                'pos': [x, y, z, 0.0, 0.0, 0.0], 
                'time_step': dt
            })
        return trajectory, dt
    

    def trajectory_circle(self, radius, target_velocity, points):
        self.circle_points, self.dt = self.generate_trajectory(radius, target_velocity, points)
        self.current_point_idx = 0
        self.is_running_circle = True

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)

        self.timer = self.create_timer(self.dt, self.run_trajectory)

    # def trajectory_circle(self, radius, target_velocity, points):

    #     self.circle_points, self.dt = self.generate_trajectory(radius, target_velocity, int(points))
        
    #     self.get_logger().info(f'quy dao: R={radius}, V={target_velocity}, dt={self.dt:.4f}s ---')

    #     # lay toa do diem dau tien quy dao moi [x, y, z, r, p, y]
    #     first_pose = self.circle_points[0]['pos']

    #     # di chuyen ve diem dau tien
    #     self.move_to_start_point(first_pose, duration=5.0)

    #     self.is_running_circle = True

    #     if self.timer is not None:
    #         self.timer.cancel()
    #         self.destroy_timer(self.timer)
    #         self.timer = None

    #     self.get_logger().info('Bat dau chay quy dao moi...')
    #     self.timer = self.create_timer(self.dt, self.run_trajectory)

    
    def get_current_leg_lengths_mm(self):
        self.get_logger().info(f'vi tri hien tai: {self.joint_positions}')
        return [float(p * self.PULSE_TO_MM) for p in self.joint_positions]
    
    def move_to_start_point(self, target_pose, duration):
        self.get_logger().info('bat dau di chuyen ve diem start point cua quy dao')

        # tinh do dai chan muc tieu diem bat dau quy dao
        T = np.array([target_pose[0], target_pose[1], target_pose[2]])
        R = self.get_rotation_matrix(target_pose[3], target_pose[4], target_pose[5])
        target_lengths = self.calculate_inverse_kinematics(T, R)

        # do dai cua xy lanh can di chuyen tinh tu home
        target_deltas = [l - self.L_LEG - self.L_HOME for l in target_lengths]

        # lay do dai hien tai
        start_deltas = self.get_current_leg_lengths_mm()
        start_deltas = [l + self.L_HOME for l in start_deltas]
        self.get_logger().info(f'do dai hien tai: {start_deltas}')

        steps = 50
        dt = duration / steps

        for i in range(steps):
            alpha = i / steps

            current_step_deltas = [
                start_deltas[j] + alpha * (target_deltas[j] - start_deltas[j])
                for j in range(6)
            ]

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = [f'joint_{k+1}' for k in range(6)]
            
            point = JointTrajectoryPoint()
            safe_deltas = [np.clip(d, -20.0, 370.0) for d in current_step_deltas]
            point.positions = [float(self.mm_to_pulse(d)) for d in safe_deltas]
            
            point.time_from_start = Duration(sec=0, nanosec=int(dt * 2.0 * 1e9)) 
            
            msg.points.append(point)
            self.publisher_.publish(msg)

            time.sleep(dt)

        self.get_logger().info('Đã đến điểm bắt đầu an toàn.')


    def run_trajectory(self):
        if(not self.is_running_circle):
            self.trajectory_circle()
            return

        if self.current_point_idx >= len(self.circle_points):
            self.get_logger().info('hoan thanh')

            self.get_logger().info(f'toa do hien tai la: {self.joint_positions}')
            self.is_running_circle = False

            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
                return
        
        p = self.circle_points[self.current_point_idx]
        dt = self.dt
        
        target_T = np.array([p['pos'][0], p['pos'][1], p['pos'][2]])
        target_R = self.get_rotation_matrix(p['pos'][3], p['pos'][4], p['pos'][5])
        lengths_mm = self.calculate_inverse_kinematics(target_T, target_R)

        if lengths_mm is not None:
            delta_mm = [l - self.L_LEG - self.L_HOME for l in lengths_mm]

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = [f'joint_{i+1}' for i in range(6)]
            
            point = JointTrajectoryPoint()
            point.positions = [float(self.mm_to_pulse(l)) for l in delta_mm]
            point.velocities = []

            buffer = 1.2
            nanosecs = int(dt * buffer * 1e9)

            point.time_from_start = Duration(
            sec=int(nanosecs // 1e9), 
            nanosec=int(nanosecs % 1e9)
            )

            msg.points.append(point)
            self.publisher_.publish(msg)
            
            self.current_point_idx += 1


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
