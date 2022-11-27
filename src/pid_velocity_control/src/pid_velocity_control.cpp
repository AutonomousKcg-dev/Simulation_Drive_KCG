#include <cmath>
#include <iostream>

#include <pid_velocity_control/pid_velocity_control.hpp>
#include "rclcpp/rclcpp.hpp"

// constructor - called at the first time
PIDvelocityControl::PIDvelocityControl()
{
  prev_accel_cmd_ = 0;
  prev_error_ = 0;
  error_integral_= 0;
  first_flag_ = true;

  time_t t0 = time(0);   // get time now
  struct tm * now = localtime( & t0 );
  char buffer [80];
  strftime (buffer,80,"%Y-%m-%d-%R.",now);
  std::string date(buffer);
  // logfile.open("log_pid_"+date);

  // logfile<<"time\t"<<"velocity\t"<<"desired_velocity\t"<<
  //   "error\t"<<"error_dot\t"<<"error_integral\t"<<
  //   "vel_ind\t"<<"d_accel_cmd\t"<<"accel_cmd \n";
  
}

int PIDvelocityControl::velToindex(double vel)
{
  for (int i =1;i<num_of_gains_;i++)
  {
    if(vel < gain_steps_[i] )
      return i-1;
  }
  return num_of_gains_-1;
}
//----------------------------------------------------------//
//compile: colcon build --packages-select pid_velocity_control

// pid controller function:
// inputs: state and desired velocity command
// output steering command acceleration command
bool PIDvelocityControl::comp_command(VehicleKinematicState & state,VehicleControlCommand in_command, float & accel, float & steer)
{
  if (first_flag_)
  {
    prev_t_ = state.header.stamp;
    first_flag_ = false;
    steer_last = 0.0F;
    angular_vel_e_last = 0.0F;
    return false;
  }
  
  double vel = double(std::sqrt(std::pow(state.state.longitudinal_velocity_mps,2)+std::pow(state.state.lateral_velocity_mps,2)));
  double vel_desired = double(in_command.velocity_mps);
  double error = vel_desired - vel;
  rclcpp::Time t = state.header.stamp;
  rclcpp::Duration  dt = t - prev_t_;
  double double_dt = dt.seconds();
   double error_dot = 0.0;
   if (double_dt> 1e-6){  
     error_dot = (error-prev_error_)/double_dt;//accel_;//
   }

  error_integral_ += error*double_dt;

  int vel_ind = velToindex(vel);

   double d_accel_cmd = kp_[vel_ind]* error+ kd_[vel_ind]* error_dot + ki_[vel_ind]* error_integral_;
   //kp_e =  (1./(1.+exp(-abs( error_)*sigmoid_k)))*kp_[vel_ind];
   //double d_accel_cmd = kp_e* error_+ kd_[vel_ind]* error_dot + ki_[vel_ind]* error_int_;
  // std::cout<< "double_dt: "<<double_dt<<" error_dot: "<<error_dot<<" error_integral_: "<<error_integral_<<std::endl;
 
  // std::cout<< "gain index: "<<vel_ind<<" kp: "<< kp_[vel_ind]<<" kd: "<<kd_[vel_ind]<<" ki: "<<ki_[vel_ind]<<std::endl;
  // std::cout<<"error: "<<error<<"d_accel_cmd: "<<d_accel_cmd<<std::endl;

//throttle_area and accelerate - max_throttle_acc
//throttle_area and deccelerate
//brake_area and accelerate
//brake_area and deccelerate

  if (prev_accel_cmd_ > 0)//throttle 
  {
      d_accel_cmd =  std::min(max_throttle_acc_/double_dt,d_accel_cmd);
      d_accel_cmd =  std::max(min_throttle_acc_/double_dt,d_accel_cmd);
  }
  else//brake 
  {
      d_accel_cmd =  std::min(max_brake_acc_/double_dt,d_accel_cmd);
      d_accel_cmd =  std::max(min_brake_acc_/double_dt,d_accel_cmd);
  } 

  // double accel_cmd = d_accel_cmd; // direct pid control
  double accel_cmd = prev_accel_cmd_ + d_accel_cmd; //pid control on change


  accel_cmd = std::min(accel_cmd,max_throttle_);
  accel_cmd = std::max(accel_cmd,min_throttle_);

  //ensure braking at full stop:
  if(vel < 0.3 && vel_desired < 0.01){
    accel_cmd = -0.25;
  }

  //set final accel:  
  accel = float(accel_cmd);
  
  prev_t_ = t;
  prev_error_ = error;
  prev_accel_cmd_ = accel_cmd;

///////////////////////////////////////////////////////
//steering control - direct steering:
steer = in_command.front_wheel_angle_rad;

float max_steer_angle = 0.7F;//std::min(0.7F, steer_last + max_d_steer);// 0.02F;//0.0117F;

// if (steer > max_steer_angle){
//   steer = max_steer_angle;
// }
// else if (steer < min_steer_angle){  
//   steer = min_steer_angle;
// }
steer = std::clamp(steer, -max_steer_angle, max_steer_angle);



steer_last = steer;

  

if (enable_log_to_file_)
{
  // logfile<<t.seconds()<<'\t'<<vel<<'\t'<<vel_desired<<'\t'<<
  //   error<<'\t'<<error_dot<<'\t'<<error_integral_<<'\t'<<
  //   vel_ind<<'\t'<<d_accel_cmd<<'\t'<<accel_cmd<<'\n';
}
  return false;
}


