#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <sstream>

#include<iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>


using Point = geometry_msgs::msg::Point;
using ColorRGBA = std_msgs::msg::ColorRGBA;


using namespace std::chrono_literals;
using std::placeholders::_1;

using TrajectoryPoint =autoware_auto_msgs::msg::TrajectoryPoint;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
using Heading = autoware_auto_msgs::msg::Complex32;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using Header = std_msgs::msg::Header;

float to_angle(Heading heading) noexcept
{
  const auto mag2 = (heading.real * heading.real) + (heading.imag * heading.imag);
  if (std::abs(mag2 - 1.0F) > 1e-5F) {
    const auto imag = 1.0F / std::sqrt(mag2);
    heading.real *= imag;
    heading.imag *= imag;
    // Don't need to touch imaginary/z part
  }
  // See:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const auto y = 2.0F *heading.real * heading.imag;
  const auto x = 1.0F - (2.0F *heading.imag * heading.imag);
  return std::atan2(y, x);
}

Marker create_points_marker(const std::string type_name, std::vector<TrajectoryPoint> &points,float z,float r, float g, float b, bool delete_marker,builtin_interfaces::msg::Time t);
class RecordTrajNode : public rclcpp::Node

{  
  
 public:
  
  bool inl=false;
    int count=0; 
  //constructor:
  RecordTrajNode()
  : Node("record_traj"),count_(0)
  {
    
      file_name = declare_parameter("record_file").get<std::string>();
        std::ifstream fw;
   //fw.open(std::string("src/control/record_traj/param/example.txt"));
      fw.open(file_name, std::ofstream::out | std::ofstream::trunc);
    RCLCPP_INFO(this->get_logger(), "file_name: %s",file_name.c_str());
      

    last_x=0.0;
    last_y=0.0;
    d_min=1;
    // //publishers:
    // pub_ctrl_cmd_ = this->create_publisher<autoware_auto_msgs::msg::VehicleControlCommand>("raw_command", 10);
     pub_marker_ = this->create_publisher<Marker>("recorded_Trajectory_Marker", 10);

 //create and open file;
   
  
    //subscribers
    sub_kinematic_state_ = this->create_subscription<VehicleKinematicState>(
      "vehicle_kinematic_state", 10, std::bind(&RecordTrajNode::on_state, this, _1));
  
 
  }
~RecordTrajNode(){
  fw.close();
}
 
private: 
  //when "VehicleKinematicState" msg is received:
  std::vector<TrajectoryPoint> traj;
  std::ofstream fw;
    float d_min;
    float last_x;
    float last_y;
     std::string file_name; 
  void on_state(const VehicleKinematicState::SharedPtr msg)
  {
    VehicleKinematicState state = *msg;   
    if(sqrt(pow(state.state.x-last_x,2)+pow(state.state.y-last_y,2))>d_min){
     write_to_file(file_name,state.state.x,state.state.y,state.state.longitudinal_velocity_mps);
    // RCLCPP_INFO(this->get_logger(), "state: %f, %f",state.state.x,state.state.y,state.state.time_from_start);
    RCLCPP_INFO(this->get_logger(), "state: %f, %f, %f",state.state.x,state.state.y,state.state.longitudinal_velocity_mps);
     
      last_x=state.state.x;
      last_y=state.state.y;
      
     
      TrajectoryPoint p;
    p.x=state.state.x;
    p.y=state.state.y;
    p.longitudinal_velocity_mps=state.state.longitudinal_velocity_mps;
    p.lateral_velocity_mps=state.state.lateral_velocity_mps;
    traj.push_back(p);
        Marker marker=create_points_marker("recorder_trajecotory",traj,0.2,0.445, 0.62, 0.8,false,state.header.stamp);
      pub_marker_->publish(marker);
    }
  
    
    
  } 
  void write_to_file(std::string file_name,float x,float y,float long_vel){
    
    std::ofstream fw(file_name, std::fstream::app);
    fw<<x<<"\t"<<y<<"\t"<<long_vel<<std::endl;
    
  }
    //subscribers:
    rclcpp::Subscription<VehicleKinematicState>::SharedPtr sub_kinematic_state_;
    rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

   size_t count_;
   

  };
  //crate point marker to plot in rviz/lgsvl
  //z=0? want on track what about delete marker? where to call this method?
Marker create_points_marker(const std::string type_name, std::vector<TrajectoryPoint> &points,float z,float r, float g, float b, bool delete_marker,builtin_interfaces::msg::Time t)
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
      

      for(size_t i = 0;i<points.size(); i++){
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RecordTrajNode>());
  rclcpp::shutdown();
  return 0;
}