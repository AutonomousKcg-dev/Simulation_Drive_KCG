#ifndef PID_CONTROLER
#define PID_CONTROLER

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
// #include <autoware_auto_msgs/msg/trajectory.hpp>
#include "rclcpp/rclcpp.hpp"
#include <fstream>


using VehicleControlCommand = autoware_auto_msgs::msg::VehicleControlCommand;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;

class PIDvelocityControl
{
  public:
    PIDvelocityControl();
    bool comp_command(VehicleKinematicState & state,VehicleControlCommand in_command, float & accel, float & steer);

    const static int num_of_gains_ = 4;
    double kp_[num_of_gains_],kd_[num_of_gains_],ki_[num_of_gains_];
    double gain_steps_[num_of_gains_];
    double max_throttle_acc_, min_throttle_acc_, max_brake_acc_, min_brake_acc_, max_throttle_,min_throttle_;
    bool vrx_simulator_;
    bool enable_log_to_file_;
  private:
    int velToindex(double vel);


    bool first_flag_;

    double error_integral_;

    rclcpp::Time prev_t_;
    float steer_last;
    float angular_vel_e_last;
    //double prev_t_;
    double prev_error_;
    double prev_accel_cmd_;
    std::ofstream logfile;

};


#endif //PID_CONTROLER