#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
import sys
import time

class SyncPositionNode(Node):
    def __init__(self):
        super().__init__('sync_pos_node')
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.goal_pub = self.create_publisher(
            JointTrajectory, 
            '/trajectory_controller/joint_trajectory', 
            10)
        
        self.current_actual_positions = None
        self.joint_names = None
        self.get_logger().info("--- Dang cho du lieu tu /joint_states ---")

        # kiem tra chac chan desired bang actual
        self.current_desired_positions = None
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/trajectory_controller/state',
            self.state_callback,
            10)

    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_actual_positions = list(msg.position)
            self.joint_names = msg.name

    def state_callback(self, msg):
        self.current_desired_positions = list(msg.desired.positions)

    def sync_controller(self):
        self.get_logger().info("--- Dang bat dau quy trinh dong bo ---")
        
        # thiet lap thoi gian cho (Timeout) 5 giay
        start_wait = self.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=5.0)

        # Doi cho den khi co du lieu actual_positions tu joint_states
        while rclpy.ok() and self.current_actual_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            elapsed_time = self.get_clock().now() - start_wait
            if elapsed_time > timeout_duration:
                self.get_logger().error("FAILURE: Khong nhan duoc du lieu tu JointState sau 5s!")
                rclpy.shutdown()
                sys.exit(1)

        # Neu da co du lieu, tien hanh gui lenh dong bo
        self.get_logger().info(f"Da nhan duoc vi tri thuc te: {self.current_actual_positions}")

        msg = JointTrajectory()
        msg.joint_names = self.joint_names if self.joint_names else [f'joint_{i+1}' for i in range(6)]

        point = JointTrajectoryPoint()
        point.positions = self.current_actual_positions
        point.velocities = [0.0] * len(self.current_actual_positions)
        point.time_from_start = Duration(sec=0, nanosec=200000000) # 0.2s
        msg.points.append(point)

        # Gui lenh va xac nhan (Thuc hien gui trong 1 giay)
        publish_start = time.time()
        tolerance = 5          #pulse
        try:
            while time.time() - publish_start < 1.0:
                msg.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_desired_positions is not None:
                errors = [abs(d - a) for d, a in zip(self.current_desired_positions, self.current_actual_positions)]
                # self.get_logger().info(f"errors: Desired da khop voi Actual (Error: {errors})")
                max_error = max(errors)
                
                if max_error <= tolerance:
                    self.get_logger().info(f"SUCCESS: Desired da khop voi Actual (Error: {max_error:.4f})")
                    sys.exit(0)
                else:
                    # self.get_logger().warn(f"Dang doi Controller cap nhat... Error hien tai: {max_error:.4f}")
                    return
            
        except Exception as e:
            self.get_logger().error(f"FAILURE: Loi khi publish quy dao: {e}")
            sys.exit(1)

def main():
    rclpy.init()
    node = SyncPositionNode()
    
    try:
        node.sync_controller()
    except Exception as e:
        node.get_logger().error(f"Loi: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()