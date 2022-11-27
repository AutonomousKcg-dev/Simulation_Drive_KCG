#include <file_trajectory_planner/file_trajectory_planner.hpp>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include <fstream>


Heading from_angle(float angle) noexcept
{
  Heading ret{};
  ret.real = std::cos(angle * 0.5F);
  ret.imag = std::sin(angle * 0.5F);
  return ret;
}

FileTrajectoryPlanner::FileTrajectoryPlanner() //std::string & file_name
{
  init_velocity = 0.0;
  final_velocity = 0.0;
  f_max = 10000;
  mass = 1000;
  vel_max = 84.0; //56;
  friction_coefficient = 2.5;
  height = 1.0;
  width = 10.0;
}

bool FileTrajectoryPlanner::init_traj(std::string file_name, bool compute_velocity, bool also_behind)
{
  //compute_velocity_flag = true;//*changed from compute_velocity*
  //changed from also_behind 
  m_also_behind = false;
  // changed from true to mach a non continues trajectory
  loop_trajectory = false; //trajectory last point is next to the first point - continues driving
  first_flag = true;
  last_index = 0;
  full_traj_vector = read_trajectory_from_file(file_name);
  return false;
}

std::vector<TrajectoryPoint>& FileTrajectoryPlanner::get_traj()
{
  return full_traj_vector;
}

float comp_distance(TrajectoryPoint &s1, TrajectoryPoint &s2)
{
  return sqrt((s1.x - s2.x) * (s1.x - s2.x) +
              (s1.y - s2.y) * (s1.y - s2.y));
}
// can be chaned to Queue instead of vector to solve finite track
std::vector<TrajectoryPoint> FileTrajectoryPlanner::read_trajectory_from_file(std::string file_name)
{
  std::ifstream infile;
  infile.open(file_name);
  if (!infile.is_open())
  {
    infile.close();
    printf("Read file Error. Cannot open the file. \n");
  }
  std::vector<TrajectoryPoint> traj_vector;
  TrajectoryPoint tp;
  for (int i = 0; i < 10000; i++) // limit path lenght
  {
    if (infile.eof())
    {
      break;
    }
    if (
        !(infile >>
          tp.x >>
          tp.y >>
          tp.longitudinal_velocity_mps))
    {
      break;
    }
    // tp.acceleration_mps2 = 1.0;
    traj_vector.push_back(tp);
  }
  infile.close();

  if (traj_vector.size() < 2)
  {
    printf("empty file!");
    return traj_vector;
  }

  for (size_t i = 0; i < traj_vector.size() - 1; i++)
  {
   
    float dx = traj_vector.at(i + 1).x - traj_vector.at(i).x;
    float dy = traj_vector.at(i + 1).y - traj_vector.at(i).y;
    float angle = std::atan2(dy, dx);
    traj_vector.at(i).heading = from_angle(angle);
    // printf("x: %f y: %f v: %f",float(traj_vector.at(i).x),float(traj_vector.at(i).y),float(traj_vector.at(i).longitudinal_velocity_mps));
  }
  traj_vector.at(traj_vector.size() - 1).heading = traj_vector.at(traj_vector.size() - 2).heading;


  // for(std::size_t i = 1; i < traj_vector.size(); i++)
  // {
  //   traj_vector.at(i).longitudinal_velocity_mps = 20.0;
  // }

  return traj_vector;
}

Trajectory FileTrajectoryPlanner::get_trajectory(KinematicState &state)
{
  Trajectory traj;
  size_t points_number = traj.CAPACITY / 2;
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"points_number: %d",points_number);

  //get closest index
  size_t current_index = get_closest_state_index(state); //std::max(get_closest_state_index(state) -1,size_t(0));

  // if (loop_trajectory)
  // {
  //   if (m_also_behind)
  //   {
  //     if (current_index < points_number)
  //     {
  //       //before start of the trajectory:
  //       size_t points_number_to_end = points_number - current_index;
  //       size_t start_index = full_traj_vector.size() - points_number_to_end;
  //       for (std::size_t i = start_index; i < start_index + points_number_to_end; i++)
  //       {
  //         traj.points.push_back(full_traj_vector.at(i));
  //       }

  //       for (std::size_t i = 0; i < points_number * 2 - points_number_to_end; i++)
  //       {
  //         traj.points.push_back(full_traj_vector.at(i));
  //       }
  //     }
  //     else
  //     {
  //       size_t start_index = current_index - points_number;
  //       for (std::size_t i = 0; i < 2 * points_number; i++)
  //         { 
  //           size_t ind = (start_index + i) % full_traj_vector.size();
  //           traj.points.push_back(full_traj_vector.at(ind));
  //         }
  //     }
  //   }

  //   else
  //   {
  //     for (std::size_t i = 0; i < traj.CAPACITY; i++)
  //     {
  //       size_t ind;
  //       if (current_index == 0 && i == 0)
  //       {
  //         ind = full_traj_vector.size() - 1; //last point
  //       }
  //       else
  //       {
  //         ind = ((current_index - 1) + i) % full_traj_vector.size();
  //       }
  //       traj.points.push_back(full_traj_vector.at(ind));
  //     }
  //   }
  // }
  // else
  // {
    for (std::size_t i = current_index; i < std::min(current_index + traj.CAPACITY, full_traj_vector.size()); i++)
      traj.points.push_back(full_traj_vector.at(i));
  //}
  float time = 0.0;
  for (std::size_t i = 1; i < traj.points.size(); i++)
  {
    float dis = comp_distance(traj.points[i - 1], traj.points[i]);
    time += dis / traj.points.at(i).longitudinal_velocity_mps;
    traj.points[i].time_from_start.sec = int(time);
    traj.points[i].time_from_start.nanosec = uint32_t((time - floor(time)) * 1e9);
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"dis: %f ,vel: %f, time: %f",dis,traj.points.at(i).longitudinal_velocity_mps,time);
  }
  //return trajectory from closest index up to 100 points

  // RCLCPP_INFO(rclcpp::get_logger("logger"),"steer: %f",(double(steer)));

  return traj;
}

float sqr_distance(TrajectoryPoint &s1, TrajectoryPoint &s2)
{
  return (s1.x - s2.x) * (s1.x - s2.x) +
         (s1.y - s2.y) * (s1.y - s2.y);
}
std::size_t FileTrajectoryPlanner::get_closest_state_index(KinematicState current_state)
{
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"full size: %d last_index: %d" ,int(full_traj_vector.size()), int(last_index));

  size_t max_steps;
  if (first_flag)
  { //at the first time search the whole trajectory
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"max_idx: %d",(max_idx));
    max_steps = full_traj_vector.size();
    last_index = 0;
    RCLCPP_INFO(rclcpp::get_logger("logger"), "max_steps: %d", (max_steps));
    first_flag = false;
  }
  else
  {
    if (loop_trajectory)
    {
      max_steps = 200;
    }
    else
    {
      max_steps = std::min(size_t(200), (full_traj_vector.size() - last_index));
    }
  }
  size_t minimum_idx = 0;
  float min_dist = 10000.0F;
  float dis = 0;
  for (size_t i = 0; i < max_steps; i++)
  {
    size_t ind;
    if (loop_trajectory)
    {
      ind = (i + last_index) % full_traj_vector.size();
    }
    else
    {
      ind = i + last_index;
    }
    dis = sqr_distance(full_traj_vector.at(ind), current_state.state);

    // RCLCPP_INFO(rclcpp::get_logger("logger"),"i: %d",(int(i)));
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"dis: %f",dis);
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"current_state: %f %f" ,current_state.state.x,current_state.state.y);
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"point: %f %f" ,full_traj_vector.at(i).x,full_traj_vector.at(i).y);
    if (dis < min_dist)
    {
      min_dist = dis;
      minimum_idx = ind;
      // RCLCPP_INFO(rclcpp::get_logger("logger"),"min_dist: %f",(min_dist));
    }
  }
  if (min_dist >= 9000)
  {
    first_flag = true;
    RCLCPP_INFO(rclcpp::get_logger("logger"), "first flag = true");
  }
  last_index = minimum_idx;
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"indx: %d",(int(last_index)));
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"distance x: %f",(double(min_dist)));

  return minimum_idx;
}
