import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from autoware_auto_msgs.msg import VehicleControlCommand
from sensor_msgs.msg import Imu, NavSatFix
from cognata_sdk_ros2.msg import GPSAdditionalData

import numpy as np
import math



def azimot(lon2,lat2,cHomeLon1,cHomeLat1):
        # lat2 = lat2 * 1000000000
        # lon2 = lon2 * 1000000000
        # cHomeLon1 = cHomeLon1 * 1000000000
        # cHomeLat1 = cHomeLat1 * 1000000000
        azimot_1 = (90 - (math.atan2(((lat2 - cHomeLat1)) , (lon2 - cHomeLon1)))* 57.2897)

        return azimot_1

class CognataRawCommand(Node):
    def __init__(self):
        super().__init__('cognata_raw_command')
        # subscriber
        self.create_subscription(
            Imu, "/cognataSDK/GPS/imu/CognataGPS", self.acceleration_callback, 10)
        # self.create_subscription(
        #     Float32, "/cognataSDK/car_command/acceleration_cmd", self.acceleration_callback, 10)
        self.create_subscription(
            Float32, "/cognataSDK/car_command/steer_cmd", self.steer_callback, 10)
        self.create_subscription(GPSAdditionalData, "/cognataSDK/GPS/info/CognataGPS", self.gps_listener, 10)
        # publisher
        self._vehicle_command_publisher = self.create_publisher(
            VehicleControlCommand, "raw_command", 10)
        self._prev_lon = float()
        self._prev_lat = float()
        self._prev_lon = 1.0
        self._prev_lat = 1.0
        self._current_lon = float()
        self._current_lat = float()
        self.counter = 0
        self._ego_speed = float()
        # timed callback
        self.create_timer(0.1, self._pub)
        self.get_deg = float()
        # message
        self._vehicle_command = VehicleControlCommand()

    def gps_listener(self, msg: GPSAdditionalData):
        # check if i am correct
        self._current_lon = msg.position.x
        self._current_lat = msg.position.y
        self._ego_speed = msg.speed
        
        
            
        
        # self.get_logger().info(str(msg.longitude))
        # self.get_logger().info(str(msg.latitude))
        
        

        
    def acceleration_callback(self, msg: Imu):
        # self.get_logger().info(str(msg.orientation.z))
        
        # x = (msg.orientation.y**2 + msg.orientation.x**2 +msg.orientation.z**2 + msg.orientation.w**2)
        # self.get_logger().info(str(x))
        self._vehicle_command.long_accel_mps2 = msg.linear_acceleration.x
        

    def steer_callback(self, msg: Float32):
        self._vehicle_command.front_wheel_angle_rad = -(msg.data * 0.325)
    
    def _pub(self):
        if self._current_lat != self._prev_lat and self._ego_speed > 1.5:
            self.get_deg = azimot(self._current_lon,self._current_lat,self._prev_lon,self._prev_lat)
        self._prev_lon = self._current_lon
        self._prev_lat = self._current_lat
        self.get_logger().info(str(self.get_deg))
        self._vehicle_command_publisher.publish(self._vehicle_command)

def main(args=None):
    rclpy.init(args=args)

    cognata_raw_command = CognataRawCommand()

    rclpy.spin(cognata_raw_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cognata_raw_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
