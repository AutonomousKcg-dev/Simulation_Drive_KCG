#ifndef FILE_PLANNER
#define FILE_PLANNER

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>

#include <vector>
#include <string>


using BoundingBoxArray = autoware_auto_msgs::msg::BoundingBoxArray;
using KinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using Heading = autoware_auto_msgs::msg::Complex32;

class FileTrajectoryPlanner
{
  public:
    FileTrajectoryPlanner();//std::string & file_name
    Trajectory get_trajectory(KinematicState & state);
    std::vector<TrajectoryPoint>& get_traj();
    std::vector<TrajectoryPoint> read_trajectory_from_file(std::string file_name);    

    bool init_traj(std::string file_name, bool compute_velocity,bool also_behind);

    float init_velocity,final_velocity;
    float f_max, mass, vel_max, friction_coefficient, height,width;
    std::string out_file_name;
    bool compute_velocity_flag;
    bool m_also_behind;
    int count;
    bool first_flag;
    std::size_t last_index;
    std::vector<TrajectoryPoint> full_traj_vector;
    std::size_t get_closest_state_index(KinematicState current_state);
    std::string file_name;
    bool loop_trajectory;


  //private:
    //int count;
    //bool first_flag;
    //std::size_t last_index;
    //std::vector<TrajectoryPoint> full_traj_vector;
    //std::size_t get_closest_state_index(KinematicState current_state);
    //std::string file_name;
    //bool loop_trajectory;
};


#endif //FILE_PLANNER
