#include <cmath>
#include <iostream>

#include <trajectory_following/trajectory_following.hpp>
#include "rclcpp/rclcpp.hpp"

// #include <tf2/convert.h>
// #include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>

// using motion::motion_common::to_angle;
// using motion::motion_common::from_angle;
// from AutowareAuto motion_common (changed):
float to_angle(Heading heading) noexcept
{
  const auto mag2 = (heading.real * heading.real) + (heading.imag * heading.imag);
  if (std::abs(mag2 - 1.0F) > 1e-5F)
  {
    const auto imag = 1.0F / std::sqrt(mag2);
    heading.real *= imag;
    heading.imag *= imag;
    // Don't need to touch imaginary/z part
  }
  // See:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const auto y = 2.0F * heading.real * heading.imag;
  const auto x = 1.0F - (2.0F * heading.imag * heading.imag);
  return std::atan2(y, x);
}
/// Basic conversion
// Heading from_angle(float angle) noexcept
// {
//   Heading ret{};
//   ret.real = std::cos(angle * RealT{0.5});
//   ret.imag = std::sin(angle * RealT{0.5});
//   return ret;
// }

float diff_angle(float alpha, float beta) // beta - alpha
{
  float phi = abs(beta - alpha);
  if (phi > 2 * M_PI)
  {
    phi = 2 * M_PI - phi;
  } // This is either the distance or 360 - distance
  float dang = phi > M_PI ? 2 * M_PI - phi : phi;
  return dang;
}

float distance(TrajectoryPoint &s1, TrajectoryPoint &s2)
{
  return float(sqrt((s1.x - s2.x) * (s1.x - s2.x) +
                    (s1.y - s2.y) * (s1.y - s2.y)));
}
float get_angle(TrajectoryPoint &s1, TrajectoryPoint &s2)
{
  return float(atan2(s2.y - s1.y, s2.x - s1.x));
}
float sqr_distance(TrajectoryPoint &s1, TrajectoryPoint &s2)
{
  return (s1.x - s2.x) * (s1.x - s2.x) +
         (s1.y - s2.y) * (s1.y - s2.y);
}

size_t find_target_index(KinematicState &state, Trajectory &trajectory, float lookahead_dist)
{
  float sqr_lookahead_dist = lookahead_dist * lookahead_dist;
  for (size_t i = 0; i < trajectory.points.size(); i++)
  {
    if (sqr_distance(trajectory.points.at(i), state.state) > sqr_lookahead_dist)
    {
      return i;
    }
  }
  return trajectory.points.size() - 1; // last point
}

TrajectoryPoint interpolate_target(KinematicState &state, Trajectory &trajectory, float d)
{
  TrajectoryPoint p;
  float sqr_lookahead_dist = d * d;
  size_t target_index = 0;
  for (target_index = 0; target_index < trajectory.points.size(); target_index++)
  {
    if (sqr_distance(trajectory.points.at(target_index), state.state) > sqr_lookahead_dist)
    {
      if (target_index == 0)
      {
        return (trajectory.points.at(target_index));
      }
      TrajectoryPoint p1 = trajectory.points.at(target_index - 1);
      TrajectoryPoint p2 = trajectory.points.at(target_index);

      float d1 = distance(p1, state.state);
      float d2 = distance(p2, state.state);
      // RCLCPP_INFO(rclcpp::get_logger("logger"),"d1: %f d2: %f", d1,d2);

      if (d2 - d1 < 0.01)
      {
        return p1;
      }
      p.x = (p2.x - p1.x) * (d - d1) / (d2 - d1) + p1.x;
      p.y = (p2.y - p1.y) * (d - d1) / (d2 - d1) + p1.y;

      return p;
    }
  }
  return trajectory.points.back(); // last point
}

// constructor - called at the first time
TrajectoryFollwing::TrajectoryFollwing()
{
}

//----------------------------------------------------------//
// void add_to_marker_array(MarkerArray & marker_array, Header & header, const std::string type_name, uint32_t type, float x_pos,float y_pos, float heading_angle, float r, float g, float b,
//      std::string frame_id = std::string("/map"),
//      float sx = 1.0, float sy = 1.0, float sz = 1.0)//bool frame_locked = false,
// {

//   Marker marker;
//   marker.header.frame_id = frame_id;
//   marker.header.stamp = header.stamp;
//   marker.ns = "marker_test_"+type_name;
//   marker.id = 0;
//   marker.type = type;
//   marker.action = Marker::ADD;
//   marker.pose.position.x = x_pos;
//   marker.pose.position.y = y_pos;
//   marker.pose.position.z = 0;
//   tf2::Quaternion quat_tf;
//   quat_tf.setRPY(0, 0, heading_angle);
//   marker.pose.orientation = tf2::toMsg(quat_tf);
//   marker.scale.x = sx;
//   marker.scale.y = sy;
//   marker.scale.z = sz;
//   marker.color.r = r;
//   marker.color.g = g;
//   marker.color.b = b;
//   marker.color.a = 1.0;
//   // marker.lifetime = 0;//rclcpp::Duration();
//   marker.frame_locked = false;//frame_locked
//   marker.text = "";
//   //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
//   marker.mesh_use_embedded_materials = false;

//   marker_array.markers.push_back(marker);
// }

// A simple implementation of a pure pursuit controller:
// computes steering and velocity commands given the trajectory and vehicle's state:
bool TrajectoryFollwing::compute_commands(
    KinematicState &state, Trajectory &trajectory,
    float &steering_command, float &velocity_command, MarkerArray &marker_array)
{

  if (trajectory.points.size() < 1)
  { // if trajectory has zero points - return
    velocity_command = 0.0F;
    steering_command = 0.0;

    return true;
  }

  float L = 3.0;                    // vehicle length
  float min_lookahead_dist = 5.0F;  // 5.0 minimal lookahead distance (l_d).
  float max_lookahead_dist = 50.0F; // maximal lookahead distance
  float max_steer_angle = 0.33F;    // limit of steering angle [rad]

  float k = 1.0; // l_d = velocity*k

  float lookahead_dist = state.state.longitudinal_velocity_mps * k;
  // limit lookahead distance to maximal and minimal value:
  //  if (lookahead_dist > max_lookahead_dist){
  //    lookahead_dist = max_lookahead_dist;
  //  }
  //  else if (lookahead_dist < min_lookahead_dist){
  //    lookahead_dist = min_lookahead_dist;
  //  }
  lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist, max_lookahead_dist);

  // search the two closest point along the trajectory to lookahead distance and intepolate the target distance:
  TrajectoryPoint target_point = interpolate_target(state, trajectory, lookahead_dist);
  // add_to_marker_array(marker_array,state.header,"sphere", Marker::SPHERE, target_point.x,target_point.y,to_angle(target_point.heading), 0.0, 0.0, 1.0, "/map");//map - relative to map

  // angle between vehicle and taget point:
  float alpha = get_angle(state.state, target_point) - to_angle(state.state.heading);

  // compute the exact lookahead distance to the target point:
  float exact_lookahead_dist = distance(state.state, target_point); // not really necessary

  // compute steering angle:
  steering_command = float(atan(2.0F * L * float(sin(alpha)) / exact_lookahead_dist));

  // limit steering angle to a maximal and minimal value:
  //  if (steering_command > max_steer_angle){
  //    steering_command = max_steer_angle;
  //  }
  //  else if (steering_command < -max_steer_angle){
  //    steering_command = -max_steer_angle;
  //  }
  steering_command = std::clamp(steering_command, -max_steer_angle, max_steer_angle);

  // follow velocity profile of the trajectory:
  size_t forward = 3; // which point determines the velocity
  if (forward + 1 < trajectory.points.size() - 1)
  {
    velocity_command = trajectory.points.at(forward + 1).longitudinal_velocity_mps;
  }
  else
  {
    velocity_command = 0.0F;
  }

  return false;
}
