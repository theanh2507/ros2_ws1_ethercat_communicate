#!/usr/bin/env python3
import rclpy
from ethercat_msgs.srv import GetSdo, SetSdo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import time

def main():
    rclpy.init()
    node = rclpy.create_node('sync_pos_node')
    
    get_client = node.create_client(GetSdo, '/ethercat_manager/get_sdo')
    set_client = node.create_client(SetSdo, '/ethercat_manager/set_sdo')
    goal_pub = node.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
    
    node.get_logger().info("--- Dang xoa vi tri cu, dong bo ve vi tri thuc te hien tai ---")

    all_success = True
    current_positions = [0.0] * 6 

    for i in range(6): 
        # 1. Doc vi tri THUC TE (0x6064)
        req_get = GetSdo.Request()  
        req_get.slave_position = i
        req_get.sdo_index = 0x6064
        req_get.sdo_data_type = 'int32'

        future = get_client.call_async(req_get)
        rclpy.spin_until_future_complete(node, future)
        
        res_get = future.result()
        if res_get.success:
            actual_pos = float(res_get.sdo_return_value)
            current_positions[i] = actual_pos

            # 2. Ghi vao Target Position (0x607A) de dong bo voi vi tri thuc te
            req_set = SetSdo.Request()
            req_set.slave_position = i
            req_set.sdo_index = 0x607A
            req_set.sdo_subindex = 0
            req_set.sdo_data_type = 'int32'
            req_set.sdo_value = str(int(actual_pos))
            
            set_future = set_client.call_async(req_set)
            rclpy.spin_until_future_complete(node, set_future)
            
            node.get_logger().info(f"Chan {i}: Thuc te {actual_pos} -> Da ghi vao Target SDO")
        else:
            all_success = False
            node.get_logger().error(f"Loi doc SDO chan {i}")

    if not all_success:
        node.get_logger().error("DONG BO THAT BAI! Dung toan bo quy trinh.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # --- GUI GOAL DE DE BUFFER CUA TRAJECTORY_CONTROLLER ---
    msg = JointTrajectory()
    msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'] 
    
    point = JointTrajectoryPoint()
    point.positions = current_positions
    # point.time_from_start = Duration(sec=0, nanosec=10000000) # 10ms de Controller kip nhan lenh
    point.time_from_start = Duration(sec=1, nanosec=0)
    
    msg.points.append(point)
    
    # Publish 5 lan lien tiep de chac chan Controller nhan duoc
    for _ in range(5):
        goal_pub.publish(msg)
        time.sleep(0.01)

    node.get_logger().info(f"Da dong bo xong ca 6 chan ve vi tri: {current_positions}")
    
    time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()