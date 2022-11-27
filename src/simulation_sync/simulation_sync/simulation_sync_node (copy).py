

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
        # self.publisher_done = self.create_publisher(
        #     Bool, "done", 0
        # )

        done = ByteMultiArray()#data = [b'\x00',b'\x00',)

        self.count = 3
        # b = byte ()

        
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


        # done = ByteMultiArray()
        # bytes_arr= int_to_bytes(self.count,4)
        # for b in bytes_arr:
        #     done.data.append(b)


        some_int = self.count
        some_bytes = some_int.to_bytes(32, sys.byteorder)
        my_bytearray = bytearray(some_bytes)
        arr = []
        for b in my_bytearray:
            arr.append(b)
        print("arr: ",arr)
        done = ByteMultiArray(data = arr)

        self.publisher_done.publish(done)

    def command_callback(self, cmd):
        # done = Bool()
        # done.data = True
        # self.publisher_done.publish(done)
        done = ByteMultiArray()
        bytes_arr= int_to_bytes(self.count,4)
        for b in bytes_arr:
            done.data.append(b)
        
        
        # done.data = self.count.to_bytes(2, 'big')
        self.count+=1 
        print(done.data)
        # done.data = True
        self.publisher_done.publish(done)

   