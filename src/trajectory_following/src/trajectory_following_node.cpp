#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"


#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>

#include <trajectory_following/trajectory_following.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;




class TrajectoryFollwingNode : public rclcpp::Node
{
public:
  TrajectoryFollwingNode()
  : Node("trajectory_following"), count_(0)
  {
    //publishers:
    pub_ctrl_cmd_ = this->create_publisher<autoware_auto_msgs::msg::VehicleControlCommand>("ctrl_cmd", 10);
    pub_marker_array_ = this->create_publisher<MarkerArray>("trajectory_following_marker_array", 10);


    //subscribers
    sub_kinematic_state_ = this->create_subscription<KinematicState>(
      "vehicle_state", 10, std::bind(&TrajectoryFollwingNode::on_state, this, _1));
    
    sub_trajectory_ = this->create_subscription<autoware_auto_msgs::msg::Trajectory>(
      "trajectory", 10, std::bind(&TrajectoryFollwingNode::on_trajectory, this, _1));
  }

private:

  void on_trajectory(const autoware_auto_msgs::msg::Trajectory::SharedPtr msg)
  {
    trajectory_ = *msg;
  }
 
  void on_state(const KinematicState::SharedPtr msg)
  {
    KinematicState state = *msg;

    float steering_command = 0,velocity_command = 0;
    MarkerArray marker_array;
    pure_pursuit_algorithm.compute_commands(state,trajectory_,steering_command,velocity_command,marker_array);

    pub_cmd(steering_command,velocity_command);
    pub_marker_array_->publish(marker_array);
  }

  void pub_cmd(float steering_command, float velocity_command)
  {
    autoware_auto_msgs::msg::VehicleControlCommand cmd;

    cmd.stamp = this->get_clock()->now();  //state_.header.stamp; //trajectory_.header.stamp;//
    cmd.velocity_mps = velocity_command;
    cmd.front_wheel_angle_rad = steering_command;//float(steer);//[rad] #-1:1 from -19:19 deg, -0.33:0.33 rad
    pub_ctrl_cmd_->publish(cmd);
  }

  //publishers:
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr pub_ctrl_cmd_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_array_;
  //subscribers:
  rclcpp::Subscription<KinematicState>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;



  TrajectoryFollwing pure_pursuit_algorithm;
  Trajectory trajectory_;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryFollwingNode>());
  rclcpp::shutdown();
  return 0;
}
