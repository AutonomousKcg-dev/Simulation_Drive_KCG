#!/usr/bin/env python3

# Copyright 2020 StreetScooter GmbH, Aachen, Germany
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#Largly modified by Gabriel hartmann

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from autoware_auto_msgs.msg import Complex32
from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import VehicleControlCommand
from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import Imu

from cognata_sdk_ros2.msg import GPSAdditionalData

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from std_msgs.msg import ByteMultiArray
import logging

import math
import sys

#import matplotlib.pyplot as pp
#import matplotlib.cm as cm
import numpy as np

import motion_model_testing_simulator.minisim as minisim
import motion_model_testing_simulator.bicycle_model as bicycleModel


# TODO(s.me) this is probably available elsewhere...
def to_angle(heading: complex) -> float:
    # Translated from motion_common's "to_angle"
    magnitude = math.sqrt(heading.real ** 2 + heading.imag ** 2)
    if abs(magnitude - 1.0) > sys.float_info.epsilon:
        heading = complex(heading.real / magnitude, heading.imag / magnitude)

    y = 2.0 * heading.real * heading.imag
    x = 1.0 - (2.0 * heading.imag * heading.imag)
    return math.atan2(y, x)

def azimot(lon2,lat2,cHomeLon1,cHomeLat1):
        # lat2 = lat2 * 1000000000
        # lon2 = lon2 * 1000000000
        # cHomeLon1 = cHomeLon1 * 1000000000
        # cHomeLat1 = cHomeLat1 * 1000000000
        azimot_1 = (-(math.atan2(((lat2 - cHomeLat1)) , (lon2 - cHomeLon1)))* 57.2897)
        azimot_1 = -(azimot_1)
        return azimot_1

# TODO(s.me) this is probably available elsewhere...
def from_angle(angle: float) -> complex:
    the_quaternion = Complex32()
    the_quaternion.real = math.cos(0.5 * angle)
    the_quaternion.imag = math.sin(0.5 * angle)
    return the_quaternion

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
class MinimalSimulatorNode(Node):
    """
    Node for testing the controller using a minisim simulation.

    This currently sends initial data to the controller node using the
    _start_test function, and then the simulation is advanced by the
    callback on receiving a command message (control_callback). This
    callback also sends a new message to the controller, keeping the
    loop going.
    """

    def __init__(self):
        super().__init__("minimal_simulator_node")

        # Simulation and geometry parameters
        self.param_sync_mode = self.declare_parameter("sync_mode",True).value #
        self.param_slave = self.declare_parameter("slave",False).value #
        self.param_publish_tf = self.declare_parameter("publish_tf",False).value
        self.param_init_x = self.declare_parameter("init_x",0.0).value
        self.param_init_y = self.declare_parameter("init_y",0.0).value
        self.param_init_velocity = self.declare_parameter("init_velocity",0.0).value
        self.param_init_heading = self.declare_parameter("init_heading",30.0).value

        self.param_state_frame = self.declare_parameter("state_frame").value
        self.param_odom_child_frame = self.declare_parameter("odom_child_frame").value
        self.param_sim_time_step = self.declare_parameter("sim_time_step_s").value
        self.output_time_s = self.declare_parameter("output_time_s").value
        self.param_cog_to_front_axle = self.declare_parameter(
            "vehicle.cog_to_front_axle"
        ).value
        self.param_cog_to_rear_axle = self.declare_parameter(
            "vehicle.cog_to_rear_axle"
        ).value
        self.param_wheelbase = self.param_cog_to_rear_axle + self.param_cog_to_front_axle


        # Publisher
        self._publisher_state = self.create_publisher(
            VehicleKinematicState, "vehicle_state", 0
        )

        self._publisher_clock = self.create_publisher(
            Clock, "/clock", 0
        )
        if self.param_publish_tf:
            self._publisher_tf = self.create_publisher(
                TFMessage, '/tf', 10
            )

        # Subscriber
        
        self.create_subscription(GPSAdditionalData, "/cognataSDK/GPS/info/CognataGPS", self.gps_listener, 10)

        self._subscriber_controls = self.create_subscription(
            VehicleControlCommand, "control_command", self.control_callback, 0
        )
        self._subscriber_done = self.create_subscription(
            ByteMultiArray, "/dds_done_reply", self.done_callback, 0
        )
        #self.create_subscription(Imu, "/cognataSDK/GPS/imu/CognataGPS", self.imu_listener, 10)
        if not self.param_slave:
            self._subscriber_reset_state = self.create_subscription(
                TrajectoryPoint, "/reset_state", self.reset_state_callback, 0
            )

        # self._subscriber_control_diag_ = self.create_subscription(
        #     ControlDiagnostic, "control_diagnostic", self.control_diag_callback, 0
        # )
        # Initializing
        self._current_command = None
        self._traj_cache = None
        self._diag_msgs = []

        #######################################################################################
        #######################################################################################
        ################################### for the angle init ################################
        #######################################################################################
        #######################################################################################

        self._prev_lon = float()
        self._prev_lat = float()
        self._prev_lon = 1.0
        self._prev_lat = 1.0
        self._current_lon = float()
        self._current_lat = float()
        self.counter = 0
        self.get_deg = float()
        self._ego_speed = float()
        # Init simulator
        # TODO(s.merkli) pick width also from ros parameters, sync with controller
        bicycle_parameters = bicycleModel.BicycleParameters(
            self.param_cog_to_front_axle, self.param_cog_to_rear_axle, width=2.0
        )
        vehicle_dynamics = bicycleModel.BicycleDynamics(bicycle_parameters)

        # We record the simulation states to memory for post-simulation evaluation
        self._memory_recorder = minisim.SimulationRecorderToMemory()

        # Create the simulation object. This does not simulate anything yet.
        self._simulator = minisim.MiniSim(
            vehicle_dynamics,
            self.param_sim_time_step,
            # listeners={"recorder": self._memory_recorder},
        )
        
        # self._current_state = bicycleModel.BicycleState(
        #     x=self.param_cog_to_rear_axle,
        #     y=0.0,
        #     v=0.0,
        #     phi=0.0,
        # )
        self._current_state = bicycleModel.BicycleState(
            x=self.param_init_x, #self.param_cog_to_rear_axle+
            y=self.param_init_y,
            v=self.param_init_velocity,
            phi=self.param_init_heading,
        )
        
        self._prev_state = None
        if self.param_publish_tf:
            tf = self.convert_bicycle_to_transform(
                # self._current_state, self.get_clock().now()
                self._current_state, Time(seconds=self._simulator.simulation_time)
            )
            self._publisher_tf.publish(tf)

        # timer for sim_tick
        if not self.param_sync_mode:
            self.timer = self.create_timer(self._simulator.step_time, self.sim_tick)
            self.timer = self.create_timer(self.output_time_s, self.output_tick)
            self.init_time = self.get_clock().now()
        elif not self.param_slave:
            self.sync_timer = self.create_timer(self.output_time_s, self.sync_timer_tick)
            self.time_step_done = False
        else:
            self.time_step_done = True
            
        self.cmd_flag = False

        # #publish for the first time
        # now = self.get_clock().now()
        # self.publish_state(now)# initial publish
        self._vehicle_state = VehicleKinematicState()

        # initiate logger
        self._mylogger = logging.getLogger("minimal_simulator_logger")
        file_handler = logging.FileHandler("minimal_simulator_log.log", "w+")
        stream_handler = logging.StreamHandler()
        log_format = logging.Formatter("%(message)s")
        file_handler.setFormatter(log_format)
        stream_handler.setFormatter(log_format)
        self._mylogger.addHandler(file_handler)
        self._mylogger.addHandler(stream_handler)
        self._mylogger.setLevel(logging.INFO)
        self._mylogger.info("Log Initiated!")

    #def imu_listener(self, msg: Imu):
    #   heading = Complex32()
    #   heading.real = math.cos((90 - math.degrees(msg.orientation.z))* 0.5)
    #   heading.imag = math.sin((90 - math.degrees(msg.orientation.z))* 0.5)
    #   self._vehicle_state.state.heading = heading

    def gps_listener(self, msg: GPSAdditionalData):
        # check if i am correct
        self._vehicle_state.state.x = msg.position.x
        self._vehicle_state.state.y = msg.position.y
        self._vehicle_state.state.longitudinal_velocity_mps = msg.speed
        self._vehicle_state.state.lateral_velocity_mps = msg.velocity_local_3d.y
        self._current_lon = msg.position.x
        self._current_lat = msg.position.y
        self._ego_speed = msg.speed

        # log position
        self._mylogger.info("X: {}, Y: {}".format(msg.position.x, msg.position.y))

    def reset_state_callback(self,state):
        # self._current_state = bicycleModel.BicycleState(
        # x=state.x, #self.param_cog_to_rear_axle+
        # y=state.y,
        # v=state.longitudinal_velocity_mps,
        # phi=to_angle(state.heading),
        # )
        self.get_logger().warn('reset_state_callback')

        self._current_state.x=state.x
        self._current_state.y=state.y
        self._current_state.v= self._vehicle_state.state.longitudinal_velocity_mps
        self._current_state.phi=to_angle(state.heading)

        if self._current_command is not None:
            self._prev_state = self._current_state
            self._current_state = self._simulator.simulate_one_timestep(
                self._current_state, self._current_command, self._simulator.simulation_time
            )
            self._simulator.simulation_time += self._simulator.step_time

            self.time_step_done = True
            self.cmd_flag = False
            now = Time(seconds=self._simulator.simulation_time)
            self.publish_state(now)

    def sync_timer_tick(self):
        # print("sync_timer_tick")
        # Trigger first output, to start controller
        if self._current_command is None:
        
            now = Time(seconds=self._simulator.simulation_time)
            # now = self.get_clock().now()
            self.publish_state(now)# initial publish
        else:
            
            if self.cmd_flag:
                # now = self.get_clock().now()#Time(seconds=self._simulator.simulation_time)
                now = Time(seconds=self._simulator.simulation_time)
                self.publish_state(now)
                self.cmd_flag = False
                self.time_step_done = False
            else:
                self.time_step_done = True

    def done_callback(self,done):
        # self.get_logger().warn('done_callback')
        if self._current_command is None:#31.12.2020
            self._current_command = bicycleModel.BicycleCommand(
            0.0, 0.0
            )
            self.get_logger().warn('not received a command, publish 0.0, 0.0 ')


            # return
        # if self.param_sync_mode:
        self.upadtes_vehicle_state()# assume that command is arrived

        if (self.time_step_done and self.cmd_flag) or self.param_slave:
            # now = self.get_clock().now()#Time(seconds=self._simulator.simulation_time)
            now = Time(seconds=self._simulator.simulation_time)
            self.publish_state(now)
            self.cmd_flag = False
            # self.time_step_done = False
        else:
            self.cmd_flag = True#cmd received - publish when time step is done


        
    def output_tick(self):
        # print("not self.param_sync_mode")

        # now = self.get_clock().now()#Time(seconds=self._simulator.simulation_time)
        now = Time(seconds=self._simulator.simulation_time)

        self.publish_state(now)


        
        
    def sim_tick(self):
        """Entrypoint to execute sim step on regular interval."""
        # get current time, to be use for publishing
        # now = Time(seconds=self._simulator.simulation_time)  # Use simulated time
        # if self.param_real_time_sim:
        #     now = self.get_clock().now()  # Use ROS time
        # self.get_logger().warn('sim_tick')
        # # Trigger first output, to start controller
        # if self._current_command is None:
        #     # initial publish
        #     self.publish_state(now)

        # elif self.param_real_time_sim:      # Realtime im update step
        #     self.upadtes_vehicle_state()
        #     self.publish_state(now)
        if self._current_command is not None:   
            self.upadtes_vehicle_state()

        # incase controller is stuck, check for timeout and plot report
        # if self.param_stop_n_report_time_s != 0 and not self.param_real_time_sim:
        #     # assuming non-realtime is at least 10 times faster, and adding 1 sec buffer
        #     assumed_timeout = (self.param_stop_n_report_time_s / 10.0) + 1.0
        #     if (self.get_clock().now() - self.init_time) > Duration(seconds=assumed_timeout):
        #         self.final_report()

    def control_callback(self, current_command_msg):
        """Store contol command and if faster than realtime, trigger simulator update."""
        self._current_command = self.convert_to_bicycle_command(current_command_msg)
        # self.cmd_flag = True#cmd received
        # if self.param_sync_mode:
        #     self.upadtes_vehicle_state()
        #     if self.time_step_done:
        #         now = self.get_clock().now()#Time(seconds=self._simulator.simulation_time)
        #         self.publish_state(now)
        #         self.time_step_done = False
        #     else:
        #         self.cmd_flag = True#cmd received


        # if not self.param_real_time_sim:
        #     self.upadtes_vehicle_state()
            # TODO(s.merkli) develop this further:
            # Check if we have to send another trajectory or just the current state?
            # Ignore the next callback if trajectory triggered?
            # Or keep it but somehow flagged?

            # Send mpc trigger again
            # self.publish_state(Time(seconds=self._simulator.simulation_time))


    def control_diag_callback(self, diag_msg):
        self._diag_msgs.append(diag_msg)

    def upadtes_vehicle_state(self):
        """Update simulator state and publish."""
        # Check if we want to continue with the simulation
        # if self.param_stop_n_report_time_s != 0:
        #     if self._simulator.simulation_time >= self.param_stop_n_report_time_s:
        #         self.final_report()
        self._prev_state = self._current_state
        # Trigger one simulation step and update current state. In externally
        # controlled simulations, we need to update the simulation time
        # ourselves as well.
        
        self._current_state.v = self._vehicle_state.state.longitudinal_velocity_mps
        self._current_state.x = self._vehicle_state.state.x
        self._current_state.y = self._vehicle_state.state.y
        # self._current_state.phi = to_angle(self._vehicle_state.state.heading)
        if self._current_lat != self._prev_lat and self._ego_speed > 1.5:
            self.get_deg = azimot(self._current_lon,self._current_lat,self._prev_lon,self._prev_lat)
        self._prev_lon = self._current_lon
        self._prev_lat = self._current_lat
        self._current_state.phi = math.radians(self.get_deg)
        #self.get_logger().info(str(self.get_deg))
        # d_vector = (self._current_state.x - self._prev_state.x,self._current_state.y - self._prev_state.y)
        # angle = calculate_rotation_angle_from_vector_to_vector(d_vector, (1,0))
        # self._current_state.phi=to_angle(angle)
        
        self._current_state = self._simulator.simulate_one_timestep(
            self._current_state, self._current_command, self._simulator.simulation_time
        )
        self._simulator.simulation_time += self._simulator.step_time

    def publish_state(self, now: Time):
        kinematic_state = self.convert_bicycle_to_vehicle_kinematic_state(
            self._current_state, now, self._prev_state, self._simulator.step_time
        )
        self._publisher_state.publish(kinematic_state)

        if not self.param_slave:
            sim_clock = Clock()
            sim_clock.clock = now.to_msg()

            self._publisher_clock.publish(sim_clock)

        if self.param_publish_tf:
            tf = self.convert_bicycle_to_transform(
                self._current_state, now
            )
            self._publisher_tf.publish(tf)


    def convert_bicycle_to_vehicle_kinematic_state(
        self, state: bicycleModel.BicycleState, now: Time,
        prev_state: bicycleModel.BicycleState, dt_sec: float
    ) -> VehicleKinematicState:
        state_msg = VehicleKinematicState()
        # Transform odom_H_cog to odom_H_base_link
        # TODO(s.merkli): Double check
        # state_msg.state.x = state.x - np.cos(state.phi) * self.param_cog_to_rear_axle
        # state_msg.state.y = state.y - np.sin(state.phi) * self.param_cog_to_rear_axle
        state_msg.state.x = state.x
        state_msg.state.y = state.y

        state_msg.state.heading = from_angle(state.phi)
        state_msg.state.longitudinal_velocity_mps = state.v
        state_msg.state.lateral_velocity_mps = 0.0  # not modeled in this
        # state_msg.state.acceleration_mps2 = 0.0  # modeled as an input
        if (self._current_command is None):
            state_msg.state.front_wheel_angle_rad = 0.0
            state_msg.state.acceleration_mps2 = 0.0
        else:
            state_msg.state.front_wheel_angle_rad = self._current_command.steering
            state_msg.state.acceleration_mps2 = self._current_command.acceleration

        state_msg.state.heading_rate_rps =  state.v*math.tan(state_msg.state.front_wheel_angle_rad)/ self.param_wheelbase
        state_msg.state.rear_wheel_angle_rad = 0.0  # not modeled in this

        state_msg.header.stamp = now.to_msg()
        state_msg.header.frame_id = self.param_state_frame

        if prev_state is not None:
            # for rear-wheel center
            state_msg.state.heading_rate_rps = state.v * math.tan(self._current_command.steering) \
                / self.param_wheelbase

            self.get_logger().warn('state.v: %f '%(state.v))
            self.get_logger().warn('prev_state.v: %f, '%(prev_state.v))
            self.get_logger().warn('dt_sec: %f '%(dt_sec))
            state_msg.state.acceleration_mps2 = (state.v - prev_state.v) / dt_sec

        return state_msg

    def convert_bicycle_to_odometry(
        self, state: bicycleModel.BicycleState, now: Time
    ) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.param_state_frame
        odom_msg.child_frame_id = self.param_odom_child_frame
        odom_msg.header.stamp = now.to_msg()

        # odom_msg.pose.pose.position.x = state.x - np.cos(state.phi) * self.param_cog_to_rear_axle
        # odom_msg.pose.pose.position.y = state.y - np.sin(state.phi) * self.param_cog_to_rear_axle

        odom_msg.pose.pose.position.x = state.x
        odom_msg.pose.pose.position.y = state.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(state.phi / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(state.phi / 2.0)

        odom_msg.twist.twist.linear .x = state.v
        odom_msg.twist.twist.linear .y = 0.0
        odom_msg.twist.twist.linear .z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        if (self._current_command is None):
            odom_msg.twist.twist.angular.z = 0.0
        else:
            odom_msg.twist.twist.angular.z = state.v \
                * math.tan(self._current_command.steering) / self.param_wheelbase

        return odom_msg

    def convert_bicycle_to_transform(
        self, state: bicycleModel.BicycleState, now: Time
    ) -> TFMessage:
        transform = TransformStamped()
        transform.header.frame_id = self.param_state_frame
        transform.header.stamp = now.to_msg()
        transform.child_frame_id = self.param_odom_child_frame

        # transform.transform.translation.x = \
        #     state.x - np.cos(state.phi) * self.param_cog_to_rear_axle
        # transform.transform.translation.y = \
        #     state.y - np.sin(state.phi) * self.param_cog_to_rear_axle
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(state.phi / 2.0)
        transform.transform.rotation.w = math.cos(state.phi / 2.0)

        return TFMessage(transforms=[transform])

    def convert_to_bicycle_command(
        self, command_msg: VehicleControlCommand
    ) -> bicycleModel.BicycleCommand:
        # TODO(s.merkli) the second parameter should be the desired _derivative_ of
        # front wheel angle, not the front wheel angle itself.
        if self._current_state.v < 0.01 and command_msg.long_accel_mps2 < 0.0 :
            command_msg.long_accel_mps2 = 0.0
        return bicycleModel.BicycleCommand(
            command_msg.long_accel_mps2, command_msg.front_wheel_angle_rad
        )

   