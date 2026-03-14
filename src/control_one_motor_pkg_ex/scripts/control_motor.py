#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ethercat_msgs.srv import SetSdo, GetSdo
import time

class HomingManager(Node):
    def __init__(self):
        super().__init__('homing_manager')
        # Tạo client cho Service Set/Get SDO
        self.set_sdo_client = self.create_client(SetSdo, '/ethercat_manager/set_sdo')
        self.get_sdo_client = self.create_client(GetSdo, '/ethercat_manager/get_sdo')
        # self.create_subscription = self.create_subscription()

    def call_set_sdo(self, slave, index, sub, dtype, value):
        req = SetSdo.Request()
        req.master_id = 0
        req.slave_position = slave
        req.sdo_index = index
        req.sdo_subindex = sub
        req.sdo_data_type = dtype
        req.sdo_value = str(value)
        return self.set_sdo_client.call_async(req)

    def execute_auto_homing(self):
        self.get_logger().info("bat dau homing...")

        # 1. Cấu hình Method (0x6098) và Mode (0x6060)
        for i in range(6):
            self.call_set_sdo(i, 0x6098, 0, 'int8', 28)  # Method 28 - Find Home (find switch low -> find zero)
            # Tốc độ tìm switch (Sub 1) và tìm zero (Sub 2)
            self.call_set_sdo(i, 0x6099, 1, 'uint32', 5000) 
            self.call_set_sdo(i, 0x6099, 2, 'uint32', 1000)
            self.call_set_sdo(i, 0x609A, 0, 'uint32', 10000)
            self.switch_to_homing_mode(i)

        time.sleep(0.1)

    def switch_to_homing_mode(self, slave):
        self.call_set_sdo(slave, 0x6040, 0, 'uint16', 6)
        time.sleep(0.1)
        self.call_set_sdo(slave, 0x6060, 0, 'int8', 6)
        time.sleep(0.2)
        self.call_set_sdo(slave, 0x6040, 0, 'uint16', 15)
        time.sleep(0.2)
        self.call_set_sdo(slave, 0x6040, 0, 'uint16', 31)

    def check_homing_status(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = HomingManager()
    
    node.execute_auto_homing()

    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()