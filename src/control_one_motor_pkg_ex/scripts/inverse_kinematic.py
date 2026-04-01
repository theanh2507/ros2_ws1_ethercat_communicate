#!/usr/bin/env python3


import os
import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray, String


class Kinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')
        
        self.joint_names = []
        self.joint_positions = [0.0] * 6
        self.joint_velocities = [0.0] * 6

        self.PULSE_PER_REV = 10000.0  # 10000 xung
        self.LEAD_SCREW_MM = 5.0      # 5 mm
        self.PULSE_TO_MM = self.LEAD_SCREW_MM / self.PULSE_PER_REV

        self.L_HOME = 30.0                      # mm
        # self.L_LEG = 1042.0                   # 930 + 54
        self.L_LEG = 1016.7178                  # do dai xy lanh giua 2 mp base va plat khi Z=982 (da bao gom 30mm di ra)
        # self.L_LEG = 1002.27
        self.Z_OFFSET = 982.0 + 173.5947        # Z_OFFSET = 982 + 173.5947 (982: z home) (khi do xy lanh di ra 20cm)

        self.circle_points = []
        self.is_running_circle = False


        self.RADIUS_BASE = 500      
        self.RADIUS_PLATFROM = 350

        self.TIME_TRAJECTORY = 0.01             # 100 Hz

        self.received_first_state = False

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # nhan pose gui tu qt
        # self.subscriptionPose = self.create_subscription(Float64MultiArray, 'pos_and_rotate_angle_topic', self.subPoseCallBack, 10)
        
        # self.get_logger().info('Inverse Kinematic Node has been started and listening to /joint_states')

        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/trajectory_controller/joint_trajectory', 
            10)
        
        self.rB = 500.0  # mm
        self.rP = 350.0  # mm
        self.phiB = np.radians(15.0)
        self.phiP = np.radians(15.0)

        def get_coords_base(r, angles):
            return np.array([[r * np.cos(a), r * np.sin(a), 0] for a in angles])

        # Z = 0 Local Coordinates
        def get_coords_platfrom(r, angles):
            return np.array([[r * np.cos(a), r * np.sin(a), 0] for a in angles])


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

        current_cpu = os.sched_getaffinity(0)
        self.get_logger().info(f'Dang chay tren core: {current_cpu}')

        self.timer = None
        ########################
        # target_T = np.array([0.0, 0.0, self.Z_OFFSET + 0.0])
        # target_R = self.get_rotation_matrix(0.0, 0.0, 0.0)
        # self.timer = self.create_timer(1.0, lambda: self.run_manual(target_T, target_R))

        ########################
        # self.trajectory_circle(radius=100.0, target_velocity=20.0, points = 20.0)

        ########################
        # kiem tra cho nhan duoc du lieu cua joint state
        # self.init_timer = self.create_timer(0.5, self.wait_for_motors_start)




        # bat dau chay quy dao khi nhan duoc lenh tu qt gui sang
        self.subscription = self.create_subscription(
            String,
            '/cmd_trajectory',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if msg.data == "run trajectory":
            self.get_logger().info('Nhan lenh: Bat dau tinh toan quy dao cho 6 truc Moons...')
            self.init_timer = self.create_timer(0.5, self.wait_for_motors_start)



    def wait_for_motors_start(self):
        if not hasattr(self, 'received_first_state') or not self.received_first_state:
            self.get_logger().warn('Dang doi du lieu tu motor (JointState)...', throttle_duration_sec=2.0)
            return

        self.init_timer.cancel()
        self.get_logger().info('Da nhan du lieu motor. Bat dau tinh toan quy dao...')
        self.trajectory_circle(radius=200.0, target_velocity=200.0)

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

        # self.get_logger().info(f'Joint Position: {self.joint_positions}')

    def subPoseCallBack(self, msg):
        target_T = np.array([msg.data[0], msg.data[1], self.Z_OFFSET + msg.data[2]])
        target_R = self.get_rotation_matrix(msg.data[3], msg.data[4], msg.data[5])
        self.run_manual(target_T, target_R)
    
    def get_rotation_matrix(self, roll, pitch, yaw):
        # xoay quanh x
        safe_roll  = np.clip(roll,  -20.0, 20.0)
        safe_pitch = np.clip(pitch, -20.0, 20.0)
        safe_yaw   = np.clip(yaw,   -60.0, 60.0)
    
        Rx = np.array([[1, 0, 0],
                   [0, np.cos(np.radians(safe_roll)), -np.sin(np.radians(safe_roll))],
                   [0, np.sin(np.radians(safe_roll)), np.cos(np.radians(safe_roll))]])
        
        # xoay quanh y
        Ry = np.array([[np.cos(np.radians(safe_pitch)), 0, np.sin(np.radians(safe_pitch))],
                    [0, 1, 0],
                    [-np.sin(np.radians(safe_pitch)), 0, np.cos(np.radians(safe_pitch))]])
        
        # xoay quanh z
        Rz = np.array([[np.cos(np.radians(safe_yaw)), -np.sin(np.radians(safe_yaw)), 0],
                    [np.sin(np.radians(safe_yaw)), np.cos(np.radians(safe_yaw)), 0],
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
        

    def run_manual(self, target_T, target_R):
                            
        self.get_logger().info(f'Ma tran xoay la: {target_R}')

        lengths_mm = self.calculate_inverse_kinematics(target_T, target_R)
        # self.get_logger().info(f'Gia tri do dai xy lanh tinh tu 2 mp base va plat: {[round(float(l), 4) for l in lengths_mm]}')
        
        if lengths_mm is not None:
            lengths_mm= [
                (l - self.L_LEG) for l in lengths_mm]
            
            formatted_lengths = [f"{l:.4f}" for l in lengths_mm]
            # self.get_logger().info(f'Gia tri do dai xy lanh can di chuyen tinh tu home la: {formatted_lengths}')
        
            real_cylinder_lengths = [f"{l + self.L_HOME :.4f}" for l in lengths_mm]

            # self.get_logger().info(f'Gia tri do dai xy lanh 6 chan: {real_cylinder_lengths}')

            self.publish_trajectory(lengths_mm, duration=5)

            if self.timer is not None:
                self.timer.cancel()

    
    def mm_to_pulse(self, target_position):
        self.target_pulse = (-target_position) / self.PULSE_TO_MM
        return self.target_pulse

    def publish_trajectory(self, lengths, duration):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [f'joint_{i+1}' for i in range(6)]
        
        point = JointTrajectoryPoint()
        
        safe_lengths = [np.clip(l, -20.0, 370.0) for l in lengths]

        point.positions = [round(float(self.mm_to_pulse(l)), 4) for l in safe_lengths]
        
        point.velocities = [0.0] * 6
        
        point.time_from_start = Duration(sec=duration, nanosec=0) 

        msg.points.append(point)
    
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Target Pulse 6 Joint : {point.positions}')

    def trajectory_circle(self, radius, target_velocity):

        self.circle_points= self.generate_trajectory_circle(radius, target_velocity)
        
        # self.get_logger().info(f'quy dao: R={radius}, V={target_velocity}, dt={self.TIME_TRAJECTORY:.4f}s ---')

        # lay toa do diem dau tien quy dao moi [x, y, z, r, p, y]
        first_pose = self.circle_points[0]['pos']

        # di chuyen ve diem dau tien
        self.move_to_start_point(first_pose, duration = 5)
        
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        self.get_logger().info('Dang doi 5s cho robot di chuyen den Start Point...')
        self.start_delay_timer = self.create_timer(5.0, self.activate_circle_timer)

    def activate_circle_timer(self):
 
        self.start_delay_timer.cancel()
        
        self.get_logger().info('Da den Start Point. Bat dau chay quy dao')
        self.current_point_idx = 0
        self.is_running_circle = True
        # sau thoi gian 5s (duration) bat dau chay quy dao
        # self.timer = self.create_timer(self.TIME_TRAJECTORY, self.run_trajectory)               # 100Hz
        self.publish_whole_trajectory()


    def publish_whole_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [f'joint_{i+1}' for i in range(6)]

        for i, p in enumerate(self.circle_points):
            # Tính toán IK cho từng điểm trong mảng
            target_T = np.array([p['pos'][0], p['pos'][1], p['pos'][2]])
            target_R = self.get_rotation_matrix(p['pos'][3], p['pos'][4], p['pos'][5])
            lengths_mm = self.calculate_inverse_kinematics(target_T, target_R)

            if lengths_mm is not None:
                delta_mm = [l - self.L_LEG for l in lengths_mm]
                safe_lengths = [np.clip(l, -20.0, 370.0) for l in delta_mm]

                point = JointTrajectoryPoint()
                point.positions = [float(self.mm_to_pulse(l)) for l in safe_lengths]
                
                # tang dan time_from_start 
                # moi diem cach nhau TIME_TRAJECTORY
                if i == 0 or i == len(self.circle_points) - 1:
                    point.velocities = [0.0] * 6
                else:
                    point.velocities = [] 

                # time_from_start nên bắt đầu từ một khoảng offset nhỏ (vd 0.1s)
                # để JTC có thời gian chuẩn bị quỹ đạo nội suy
                total_nanosecs = int((i + 1000) * self.TIME_TRAJECTORY * 1e9) 
                point.time_from_start = Duration(
                    sec=int(total_nanosecs // 1e9),
                    nanosec=int(total_nanosecs % 1e9)
                )
                msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Da gui quy dao gom {len(msg.points)} diem (gui tat cac mang quy dao cho joint trajectory controller)')


    def generate_trajectory_circle(self, radius, target_velocity):
        duration = (2 * math.pi * radius) / target_velocity
        self.get_logger().info(f'thoi gian hoan thanh 1 quy dao la: {duration}')

        if duration < 2.0: 
            duration = 2.0
            self.get_logger().warn(f'Thoi gian hoan thanh quy dao qua ngan (duoi 2s). Tu dong dieu chinh len 2s de dam bao robot co the theo kip quy dao.') 

        
        trajectory = []

        point_count_test = duration / self.TIME_TRAJECTORY
        self.get_logger().info(f'so point la: {point_count_test}')

        for i in range(abs(int(point_count_test))):
            t = i * self.TIME_TRAJECTORY                             # thoi gian di den diem thu i 
            theta = (2 * math.pi * t) / duration
            
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            z = self.Z_OFFSET
            
            trajectory.append({
                'pos': [x, y, z, 0.0, 0.0, 0.0]
            })
        return trajectory
    

    def move_to_start_point(self, target_positon, duration):
        target_T = np.array([target_positon[0], target_positon[1], target_positon[2]])
        target_R = self.get_rotation_matrix(target_positon[3], target_positon[4], target_positon[5])

        lengths_mm = self.calculate_inverse_kinematics(target_T, target_R)
        # self.get_logger().info(f'Gia tri do dai xy lanh tinh tu 2 mp base va plat: {[round(float(l), 4) for l in lengths_mm]}')
        
        if lengths_mm is not None:
            lengths_mm= [
                (l - self.L_LEG) for l in lengths_mm]
            
            formatted_lengths = [f"{l:.4f}" for l in lengths_mm]
            # self.get_logger().info(f'Gia tri do dai xy lanh can di chuyen tinh tu home la: {formatted_lengths}')
        
            real_cylinder_lengths = [f"{l + self.L_HOME :.4f}" for l in lengths_mm]

            # self.get_logger().info(f'Gia tri do dai xy lanh 6 chan: {real_cylinder_lengths}')
            self.get_logger().info("bat dau di chuyen den diem start point")
            self.publish_trajectory(lengths_mm, duration)

    def run_trajectory(self):
        if(not self.is_running_circle):
            self.trajectory_circle(radius=100.0, target_velocity=20.0)
            return

        if self.current_point_idx + 1 >= len(self.circle_points):
            # self.get_logger().info('hoan thanh')
            self.current_point_idx = 0

            self.is_running_circle = False

            # huy timer de chay 1 lan 
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
                return
            
        p = self.circle_points[self.current_point_idx]

        roll  = np.clip(p['pos'][3], -20.0, 20.0)
        pitch = np.clip(p['pos'][4], -20.0, 20.0)
        yaw   = np.clip(p['pos'][5], -60.0, 60.0)
        
        target_T = np.array([p['pos'][0], p['pos'][1], p['pos'][2]])
        target_R = self.get_rotation_matrix(roll, pitch, yaw)
        lengths_mm = self.calculate_inverse_kinematics(target_T, target_R)


        if lengths_mm is not None:
            delta_mm = [l - self.L_LEG for l in lengths_mm]

            safe_lengths = [np.clip(l, -20.0, 370.0) for l in delta_mm]

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = [f'joint_{i+1}' for i in range(6)]
            
            point = JointTrajectoryPoint()
            point.positions = [float(self.mm_to_pulse(l)) for l in safe_lengths]
            point.velocities = []

            nanosecs = int(self.TIME_TRAJECTORY * 1 *1e9)

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
