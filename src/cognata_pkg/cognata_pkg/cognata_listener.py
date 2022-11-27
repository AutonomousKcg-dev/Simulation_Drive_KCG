import rclpy
from rclpy.node import Node

from cognata_sdk_ros2.msg import GPSAdditionalData
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import Complex32
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
import math

import numpy as np

def calculate_rotation_angle_from_vector_to_vector(a,b):
    """ return rotation angle from vector a to vector b, in degrees.

    Args:
        a : np.array vector. format (x,y)
        b : np.array vector. format (x,y)

    Returns:
        angle [float]: degrees. 0~360
    """

    unit_vector_1 = a / np.linalg.norm(a)
    unit_vector_2 = b / np.linalg.norm(b)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    angle = angle/ np.pi * 180


    c = np.cross(b,a)

    if c>0:
        angle +=180
    

    return angle

class CognataListener(Node):

    def __init__(self):
        super().__init__('cognata_listener')
        # subsscriber
        self.create_subscription(GPSAdditionalData, "/cognataSDK/GPS/info/CognataGPS", self.gps_listener, 10)
        self.create_subscription(Imu, "/cognataSDK/GPS/imu/CognataGPS", self.imu_listener, 10)
        self.create_subscription(Float32, "/cognataSDK/car_command/steer_cmd", self.front_steer_angle_listener, 10)

        # publisher
        self._vehicle_state_publisher = self.create_publisher(VehicleKinematicState, "vehicle_state", 10)
        # self.create_timer(0.1, self._pub)

        # state message
        self._vehicle_state = VehicleKinematicState()

        # booleans for sync
        self._imu_received = False
        self._gps_received = False
        self._front_angle_received = False

    # def gps_listener(self, msg: GPSAdditionalData):
    #     # check if i am correct
    #     self._vehicle_state.state.x = msg.position.x
    #     self._vehicle_state.state.y = msg.position.y
    #     self._vehicle_state.state.longitudinal_velocity_mps = msg.velocity_local_3d.x
    #     self._vehicle_state.state.lateral_velocity_mps = msg.velocity_local_3d.y

        # # calculate heading 
        # angle = calculate_rotation_angle_from_vector_to_vector((1, 0), (self._vehicle_state.state.x, self._vehicle_state.state.y))
        # heading = Complex32()
        # heading.real = math.cos(angle * 0.5)
        # heading.imag = math.sin(angle * 0.5)
        # self._vehicle_state.state.heading = heading
    
    # def imu_listener(self, msg: Imu):
    #     self._vehicle_state.state.acceleration_mps2 = msg.linear_acceleration.x

    # def front_steer_angle_listener(self, msg: Float32):
    #     self._vehicle_state.state.front_wheel_angle_rad = msg.data

    # def _pub(self):
    #     self._vehicle_state_publisher.publish(self._vehicle_state)

def main(args=None):
    rclpy.init(args=args)

    cognata_listener = CognataListener()

    rclpy.spin(cognata_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cognata_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()