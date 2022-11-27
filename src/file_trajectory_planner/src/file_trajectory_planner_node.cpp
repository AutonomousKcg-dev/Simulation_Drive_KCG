#include <string>
#include <algorithm>
#include <iterator>
#include "rclcpp/rclcpp.hpp"

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <file_trajectory_planner/file_trajectory_planner.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
using Point = geometry_msgs::msg::Point;
using ColorRGBA = std_msgs::msg::ColorRGBA;
using String = std_msgs::msg::String;

using std::placeholders::_1;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
using Marker = visualization_msgs::msg::Marker;
Marker create_points_marker(const std::string type_name, std::vector<TrajectoryPoint> &points, float z, float r, float g, float b, bool delete_marker, builtin_interfaces::msg::Time t);

class FileTrajectoryPlannerNode : public rclcpp::Node
{
public:
  FileTrajectoryPlannerNode()
      : Node("file_trajectory_planner"), count_(0)
  {
    // to change into test.txt
    const std::string file_name = declare_parameter("trajectory_file_name1").get<std::string>();
    const std::string file_name2 = declare_parameter("trajectory_file_name2").get<std::string>();

    lanes.push_back(file_trajextory_planner_.read_trajectory_from_file(file_name));
    lanes.push_back(file_trajextory_planner_.read_trajectory_from_file(file_name2));

    RCLCPP_INFO(rclcpp::get_logger("logger"), "file_name %s", file_name.c_str());
    const bool compute_velocity = declare_parameter("compute_velocity").get<bool>();
    declare_parameter("also_behind", false);
    // const bool also_behind = declare_parameter("also_behind",true).get<bool>();
    int count_lane = 0;

    bool also_behind;
    this->get_parameter("also_behind", also_behind);
    file_trajextory_planner_.init_traj(file_name, compute_velocity, also_behind);

    // publishers:
    pub_trajectory_ = this->create_publisher<autoware_auto_msgs::msg::Trajectory>("trajectory", 10);
    pub_marker_ = this->create_publisher<Marker>("trajectory_marker", 10);
    
    // subscribers
    sub_kinematic_state_ = this->create_subscription<VehicleKinematicState>(
        "vehicle_state", 10, std::bind(&FileTrajectoryPlannerNode::on_state, this, _1));

    sub_lane = this->create_subscription<String>(
        "lane_switch", 10, std::bind(&FileTrajectoryPlannerNode::set_traj, this, _1));
  }

private:
  void on_state(const VehicleKinematicState::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "on state");
    state_ = *msg;
    Trajectory traj = file_trajextory_planner_.get_trajectory(state_);
    traj.header = state_.header;
    pub_trajectory_->publish(traj);

    std::vector<TrajectoryPoint> traj_marker;
    for (size_t i = 0; i < traj.points.size(); i++)
    {
      traj_marker.push_back(traj.points[i]);
    }
    Marker marker = create_points_marker("recorder_trajecotory", traj_marker, 0.2, 0.445, 0.62, 0.8, false, state_.header.stamp);
    pub_marker_->publish(marker);
  }

  TrajectoryPoint weighted_avg_(TrajectoryPoint &point1, TrajectoryPoint &point2, float p)
  {
    TrajectoryPoint weighted_point;
    weighted_point.x = point1.x * p + point2.x * (1 - p);
    weighted_point.y = point1.y * p + point2.y * (1 - p);

    return weighted_point;
  }

  TrajectoryPoint weighted_avg(TrajectoryPoint &point1, TrajectoryPoint &point2, float p)
  {
    if (p < 0 || p > 1.0)
    {
      throw std::runtime_error("p value out of range!\n");
    }
    return weighted_avg_(point1, point2, p);
  }

  void set_traj(String::SharedPtr msg)
  {
    String file_name = *msg;
    if (file_name.data.compare("RIGHT") == 0)
    {
      std::cout << "here" << std::endl;
      // this->count_lane = 1;
      if(this->count_lane < lanes.size() - 1){
          this->count_lane++;
      }
      std::vector<TrajectoryPoint> next_lane = this->lanes.at(this->count_lane);
      // find the number of the points
      float p_ = 0.1;
      size_t old_lane_index = file_trajextory_planner_.get_closest_state_index(this->state_);
      file_trajextory_planner_.get_traj() = next_lane;
      size_t new_lane_index = file_trajextory_planner_.get_closest_state_index(this->state_);
      // file_trajextory_planner_.get_traj().erase(file_trajextory_planner_.get_traj().begin(), file_trajextory_planner_.get_traj().begin() + new_lane_index + 2);
      file_trajextory_planner_.get_traj().erase(file_trajextory_planner_.get_traj().begin(), file_trajextory_planner_.get_traj().begin() + 25);
      std::cout << "[DEBUG] - Next Index: " << new_lane_index << "\n";
 
    }
    else if (file_name.data.compare("LEFT") == 0)
    {
      // this->count_lane = 0;
      if(this->count_lane > 0){
          this->count_lane--;
      }
      std::vector<TrajectoryPoint> next_lane = this->lanes.at(this->count_lane);
      // find the number of the points
      float p_ = 0.1;
      size_t old_lane_index = file_trajextory_planner_.get_closest_state_index(this->state_);
      file_trajextory_planner_.get_traj() = next_lane;
      size_t new_lane_index = file_trajextory_planner_.get_closest_state_index(this->state_);
      // file_trajextory_planner_.get_traj().erase(file_trajextory_planner_.get_traj().begin(), file_trajextory_planner_.get_traj().begin() + new_lane_index + 2);
      file_trajextory_planner_.get_traj().erase(file_trajextory_planner_.get_traj().begin(), file_trajextory_planner_.get_traj().begin() + 25);
      std::cout << "[DEBUG] - Next Index: " << new_lane_index << "\n";

    }
  }

  // publishers:
  rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  // subscribers:
  rclcpp::Subscription<VehicleKinematicState>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<String>::SharedPtr sub_lane;

  FileTrajectoryPlanner file_trajextory_planner_;
  VehicleKinematicState state_;
  std::vector<std::vector<TrajectoryPoint>> lanes;
  size_t count_;
  int count_lane;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FileTrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

Marker create_points_marker(const std::string type_name, std::vector<TrajectoryPoint> &points, float z, float r, float g, float b, bool delete_marker, builtin_interfaces::msg::Time t)
{
  Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = t;
  marker.ns = type_name;
  marker.type = Marker::POINTS;
  if (delete_marker)
  {
    marker.action = Marker::DELETE;
  }
  else
  {
    marker.action = Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    ColorRGBA c;
    c.a = 1.0;
    c.b = b;
    c.r = r;
    c.g = g;

    for (size_t i = 0; i < points.size(); i++)
    {
      Point p;
      p.x = points[i].x;
      p.y = points[i].y;
      p.z = z;
      marker.points.push_back(p);
      marker.colors.push_back(c);
    }
  }

  return marker;
}