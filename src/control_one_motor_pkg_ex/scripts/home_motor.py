#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ethercat_msgs.srv import SetSdo, GetSdo
import time

class HomingManager(Node):
    def __init__(self):
        super().__init__('homing_manager')

        self.set_sdo_client = self.create_client(SetSdo, '/ethercat_manager/set_sdo')
        self.get_sdo_client = self.create_client(GetSdo, '/ethercat_manager/get_sdo')


    def call_set_sdo(self, slave, index, sub, dtype, value):
        req = SetSdo.Request()
        req.master_id = 0
        req.slave_position = slave
        req.sdo_index = index
        req.sdo_subindex = sub
        req.sdo_data_type = dtype
        req.sdo_value = str(value)
        return self.set_sdo_client.call_async(req)

    def call_set_sdo_sync(self, slave, index, sub, dtype, value):
        req = SetSdo.Request()
        req.master_id = 0
        req.slave_position = slave
        req.sdo_index = index
        req.sdo_subindex = sub
        req.sdo_data_type = dtype
        req.sdo_value = str(value)
        
        while not self.set_sdo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        future = self.set_sdo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self.get_logger().info(f"Set SDO Slave {slave} Index 0x{index:X} Sub {sub} to {value} - Result: {future.result().success}")
        
        return future.result()

    def execute_auto_homing(self):
        self.get_logger().info("Bắt đầu cấu hình Homing...")

        for i in range(6):
            self.get_logger().info(f"Configuring Slave {i}...")
            self.call_set_sdo_sync(i, 0x6098, 0, 'int8', 25)
            self.call_set_sdo_sync(i, 0x6099, 1, 'uint32', 20000)
            self.call_set_sdo_sync(i, 0x6099, 2, 'uint32', 5000)
            self.call_set_sdo_sync(i, 0x609A, 0, 'uint32', 10000)

            # Homing Mode
            self.call_set_sdo_sync(i, 0x6060, 0, 'int8', 6)

        for i in range(6):
            self.call_set_sdo_sync(i, 0x6040, 0, 'uint16', 6)  # Shutdown
        time.sleep(0.1)
        for i in range(6):
            self.call_set_sdo_sync(i, 0x6040, 0, 'uint16', 15) # Enable

        self.get_logger().info("6 truc ve home...")
        for i in range(6):
            self.call_set_sdo(i, 0x6040, 0, 'uint16', 31) # Start Homing



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