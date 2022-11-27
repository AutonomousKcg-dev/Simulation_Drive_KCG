
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <pid_velocity_control/pid_velocity_control.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;



class PIDvelocityControlNode : public rclcpp::Node
{
public:
  PIDvelocityControlNode()
  : Node("pid_velocity_control"), count_(0)
  {

    in_command_.velocity_mps = 0.0F;
    // pub_on_cmd = false;
    first_flag_ = true;
    //publishers:
    pub_ctrl_cmd_ = this->create_publisher<VehicleControlCommand>("raw_command_master", 10);

    //subscribers
    sub_kinematic_state_ = this->create_subscription<VehicleKinematicState>(
      "vehicle_state", 10, std::bind(&PIDvelocityControlNode::on_state, this, _1));
    sub_in_command_ = this->create_subscription<VehicleControlCommand>(
      "vehicle_command", 10, std::bind(&PIDvelocityControlNode::on_cmd, this, _1));


    std::vector<double> kp,kd,ki,gain_steps;

    rclcpp::Parameter kp_param,kd_param,ki_param,gain_steps_param;
    this->declare_parameter("kp");
    this->declare_parameter("kd");
    this->declare_parameter("ki");
    this->declare_parameter("gain_steps");
    // this->set_parameters({rclcpp::Parameter("kp", std::vector<double>({1}))});

    this->get_parameter("kp", kp_param);
    kp = kp_param.as_double_array();
    std::copy(kp.begin(), kp.end(), PIDcontroller.kp_);

    this->get_parameter("kd", kd_param);
    kd = kd_param.as_double_array();
    std::copy(kd.begin(), kd.end(), PIDcontroller.kd_);

    this->get_parameter("ki", ki_param);
    ki = ki_param.as_double_array();
    std::copy(ki.begin(), ki.end(), PIDcontroller.ki_);

    this->get_parameter("gain_steps", gain_steps_param);
    gain_steps = gain_steps_param.as_double_array();
    std::copy(gain_steps.begin(), gain_steps.end(), PIDcontroller.gain_steps_);


    PIDcontroller.max_throttle_acc_= declare_parameter("max_throttle_acc").get<double>();
    PIDcontroller.min_throttle_acc_= declare_parameter("min_throttle_acc").get<double>();
    PIDcontroller.max_brake_acc_= declare_parameter("max_brake_acc").get<double>();
    PIDcontroller.min_brake_acc_= declare_parameter("min_brake_acc").get<double>();
    PIDcontroller.max_throttle_= declare_parameter("max_throttle").get<double>();
    PIDcontroller.min_throttle_= declare_parameter("min_throttle").get<double>();

    PIDcontroller.vrx_simulator_ = declare_parameter("vrx_simulator").get<bool>();
    PIDcontroller.enable_log_to_file_ = declare_parameter("enable_log_to_file").get<bool>();


  }

private:
  void on_cmd(const VehicleControlCommand::SharedPtr msg)
  {
    in_command_ = *msg;
    double double_t = rclcpp::Time(msg->stamp).seconds();// + 0.002;
    double double_current_t = rclcpp::Time(current_t_).seconds();
 
    if (double_current_t < double_t || first_flag_)
    {

      float accel = 0, steer = 0;
      PIDcontroller.comp_command(state_,in_command_,accel,steer);
      pub_cmd(accel,steer);
      current_t_ = msg->stamp; 
      first_flag_ = false;
    }

  }


  void on_state(const VehicleKinematicState::SharedPtr msg)
  {
    state_ = *msg;
    float accel = 0, steer = 0;

    double double_t = rclcpp::Time(msg->header.stamp).seconds() + 0.002;

      PIDcontroller.comp_command(state_,in_command_,accel,steer);
      pub_cmd(accel,steer);

  }

  void pub_cmd(float accel, float steer)
  {
    autoware_auto_msgs::msg::VehicleControlCommand cmd;
    cmd.stamp = state_.header.stamp;
    cmd.long_accel_mps2 = accel;//float(accel);//[m/s^2]
    cmd.front_wheel_angle_rad = steer;//float(steer);//[rad] #-1:1 from -19:19 deg, -0.33:0.33 rad
    pub_ctrl_cmd_->publish(cmd);
  }

  //publishers:
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr pub_ctrl_cmd_;
  //subscribers:
  rclcpp::Subscription<VehicleKinematicState>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<VehicleControlCommand>::SharedPtr sub_in_command_;

  
  builtin_interfaces::msg::Time current_t_;
  bool first_flag_;

  PIDvelocityControl PIDcontroller;
  VehicleKinematicState state_;
  VehicleControlCommand in_command_;
  VehicleControlCommand out_command_;
  bool pub_on_cmd;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDvelocityControlNode>());
  rclcpp::shutdown();
  return 0;
}
