#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ethercat_msgs.srv import GetSdo, SetSdo
from std_msgs.msg import String 
import time

class StatusMotor(Node):
    def __init__(self):
        super().__init__('status_motor')
        self.get_sdo_client = self.create_client(GetSdo, '/ethercat_manager/get_sdo')
        self.set_sdo_client = self.create_client(SetSdo, '/ethercat_manager/set_sdo')

        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.listener_callback,
            10)
        self.get_logger().info("Node StatusMotor đã sẵn sàng nhận lệnh...")

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Nhận lệnh từ Qt: {command}")
        self.execute_mode(command)

    def call_set_sdo_async(self, slave, index, sub, dtype, value):
            req = SetSdo.Request()
            req.master_id = 0
            req.slave_position = slave
            req.sdo_index = index
            req.sdo_subindex = sub
            req.sdo_data_type = dtype
            req.sdo_value = str(value)
            return self.set_sdo_client.call_async(req)

    def execute_mode(self, mode):
        futures = []
        val = 0
        if mode == 'enable': val = 15
        elif mode == 'disable': val = 6
        elif mode == 'fault_reset': val = 128

        self.get_logger().info(f"Đang kích hoạt đồng thời 6 trục giá trị: {val}")

        for i in range(6):
            f = self.call_set_sdo_async(i, 0x6040, 0, 'uint16', val)
            futures.append(f)

        # dung 0.5s doi future hoan thanh
        start_time = time.time()
        while time.time() - start_time < 0.5:
            rclpy.spin_once(self, timeout_sec=0.01)
            if all(f.done() for f in futures):
                break

        self.get_logger().info("Đã gửi lệnh xong cho toàn bộ 6 trục.")
    
    # enable lan luot
    # def call_set_sdo_sync(self, slave, index, sub, dtype, value):
    #     req = SetSdo.Request()
    #     req.master_id = 0
    #     req.slave_position = slave
    #     req.sdo_index = index
    #     req.sdo_subindex = sub
    #     req.sdo_data_type = dtype
    #     req.sdo_value = str(value)
        
    #     while not self.set_sdo_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('Service not available, waiting...')

    #     future = self.set_sdo_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
    #     # self.get_logger().info(f"Set SDO Slave {slave} Index 0x{index:X} Sub {sub} to {value} - Result: {future.result().success}")
        
    #     return future.result()
    
    # def enable_motor(self, slave):
    #     self.get_logger().info(f"enable motor")
    #     self.call_set_sdo_sync(slave, 0x6040, 0, 'uint16', 15)

    # def disable_motor(self, slave):
    #     self.call_set_sdo_sync(slave, 0x6040, 0, 'uint16', 6)

    # def fault_reset(self, slave):
    #     self.call_set_sdo_sync(slave, 0x6040, 0, 'uint16', 128)



def main(args=None):
    rclpy.init(args=args)
    node = StatusMotor()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()