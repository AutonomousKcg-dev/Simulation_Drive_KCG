

from rclpy.node import Node

from autoware_auto_msgs.msg import VehicleControlCommand

from std_msgs.msg import Bool
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Byte

import sys
def int_to_bytes(val, num_bytes):
    return [(val & (0xff << pos*8)) >> pos*8 for pos in reversed(range(num_bytes))]


class SimulationSyncNode(Node):

    def __init__(self):
        super().__init__("simulation_sync_node")
        # Publisher
        # self.publisher_command = self.create_publisher(
        #     VehicleControlCommand, "vehicle_command", 0
        # )

        self.done = ByteMultiArray()#data = [b'\x00',b'\x00',)

        self.count = 3
        # b = byte ()
        self.first_flag = True
        
        # done.data.append(b'\x00')
        # done.data.append(b'\x01')
        # print("done ", done.data)

        # # print("b: ",b)
        
        # print(int_to_bytes(self.count,4))
        # done.data =  int_to_bytes(self.count,4)#bytearray(self.count.to_bytes(2, 'big'))
        # print(done.data)

        self.publisher_done = self.create_publisher(
            ByteMultiArray, "dds_done_reply", 0
        )

        # Subscriber
        self.command_sub = self.create_subscription(
            VehicleControlCommand, "vehicle_command", self.command_callback, 0
        )

        # self.done_sub = self.create_subscription(
        #     ByteMultiArray, "/vrx/dds_done", self.done_callback, 0
        # )
        # self.bool_done_sub = self.create_subscription(
        #     Bool, "bool_done", self.bool_done_callback, 0
        # )
        

        # done = ByteMultiArray()
        # bytes_arr= int_to_bytes(self.count,4)
        # for b in bytes_arr:
        #     done.data.append(b)


        # some_int = self.count
        # some_bytes = some_int.to_bytes(32, sys.byteorder)
        # my_bytearray = bytearray(some_bytes)
        # arr = []
        # for b in my_bytearray:
        #     arr.append(b)
        # print("arr: ",arr)
        # done = ByteMultiArray(data = arr)

        # self.publisher_done.publish(self.done)

    # def bool_done_callback(self, msg):
    #     self.publisher_done.publish(self.done)

    def done_callback(self, msg):
        # self.get_logger().warn('done_callback')

        # self.done = msg
        if self.first_flag:
            self.first_flag = False
                
            # self.get_logger().warn('done_callback1')

            # print(self.done)
            # cmd = VehicleControlCommand()
            # # cmd.velocity_mps = 87.0
            # cmd.long_accel_mps2 = 1.0
            # cmd.front_wheel_angle_rad = 0.0
            # self.publisher_command.publish(cmd)
            self.publisher_done.publish(self.done)
            self.publisher_done.publish(self.done)


    def command_callback(self, cmd):
        # self.get_logger().warn('command_callback')

        # done = Bool()
        # done.data = True
        # self.publisher_done.publish(done)
        # done = ByteMultiArray()
        # bytes_arr= int_to_bytes(self.count,4)
        # for b in bytes_arr:
        #     done.data.append(b)
        
        
        # # done.data = self.count.to_bytes(2, 'big')
        # self.count+=1 
        # print(done.data)
        # done.data = True
        self.publisher_done.publish(self.done)

   