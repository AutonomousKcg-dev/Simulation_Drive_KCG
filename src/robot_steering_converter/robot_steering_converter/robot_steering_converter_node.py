#!/usr/bin/env python3

from rclpy.node import Node

# from autoware_auto_msgs.msg import ControlDiagnostic
from autoware_auto_msgs.msg import VehicleControlCommand

from geometry_msgs.msg import Twist



class RobotSteeringConverter(Node):

    def __init__(self):
        super().__init__("robot_steering_converter_node")      
        self._subscriber_control = self.create_subscription(
            Twist, "cmd_vel", self.control_callback, 0
        )
        self._publisher_converted_control = self.create_publisher(
            VehicleControlCommand, "vehicle_command", 0
        )#"control_command"

   
    def control_callback(self, twist_msg):
        conv_cmd = VehicleControlCommand()
        conv_cmd.front_wheel_angle_rad = twist_msg.angular.z# / 10.0   
        conv_cmd.velocity_mps = twist_msg.linear.x
        self._publisher_converted_control.publish(conv_cmd)

   