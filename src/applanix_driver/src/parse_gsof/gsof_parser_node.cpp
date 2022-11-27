#include "rclcpp/rclcpp.hpp"

//#include "nmea2tfpose_core.h"

// #include <additional_msgs/msg/sentence.hpp> // nmea sentence msg
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
// #include <geometry_msgs/msg/point32.hpp>
// #include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //not hpp!

// #include <time_utils/time_utils.hpp>
// #include <motion_common/config.hpp>

// #include <vrxperience_msgs/msg/gps.hpp>


// #include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/AccelStamped.h>
// #include <std_msgs/UInt32.h>
// #include <tf/transform_broadcaster.h>

#include <parse_gsof/gsofParser.hpp>
#include <parse_gsof/geo_pos_conv.hpp>

using std::placeholders::_1;
// using Sentence = autoware_auto_msgs::msg::Sentence;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;

using PoseStamped = geometry_msgs::msg::PoseStamped;
// using s = additional_msgs::msg::Sentence;
// using GPS = vrxperience_msgs::msg::GPS;


double deg2rad(double deg_val)
{
  return deg_val * M_PI / 180.0;
}
float deg2rad(float deg_val)
{
  return deg_val * float(M_PI) / 180.0F;
}
// void publishTime(ros::Publisher pub_week,ros::Publisher pub_time ,unsigned long week, unsigned long gps_time)
// {
//   pub_week.publish(week);
//   pub_time.publish(gps_time);
// }

// void publishPoseStamped(ros::Publisher pub,ros::Time current_time,double x,double y,double z, double roll, double pitch, double yaw)
// {
//   geometry_msgs::PoseStamped pose;
//   pose.header.frame_id = "map";
//   pose.header.stamp = current_time;
//   pose.pose.position.x = y;
//   pose.pose.position.y = x;
//   pose.pose.position.z = z;
//   //ROS_WARN_STREAM("x: "<<pose.pose.position.x<<" y: "<<pose.pose.position.y<<" z: "<<pose.pose.position.z);
//   pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
//   pub.publish(pose);
// }

// void publishTF(tf::TransformBroadcaster br,ros::Time current_time,double x,double y,double z, double roll, double pitch, double yaw)
// {
//   tf::Transform transform;
//   transform.setOrigin(tf::Vector3(y, x, z));
//   tf::Quaternion quaternion;
//   quaternion.setRPY(roll, pitch, yaw);
//   transform.setRotation(quaternion);
//   br.sendTransform(tf::StampedTransform(transform, current_time, "gnss_frame","gps" ));
// }
// void publishVelStamped(ros::Publisher vel_pub,ros::Time current_time,double angular_x,double angular_y,double angular_z, double linear_x, double linear_y, double linear_z,double total_speed)
// {
//   //geometry_msgs::Twist twist;
//   //twist.
//   geometry_msgs::TwistStamped ts;
//   ts.header.stamp = current_time;
//   ts.twist.angular.x = angular_x;
//   ts.twist.angular.y = angular_y;
//   ts.twist.angular.z = angular_z;
//   // ts.twist.linear.x = linear_x;
//   // ts.twist.linear.y = linear_y;
//   // ts.twist.linear.z = linear_z;
//   ts.twist.linear.x = total_speed;
//   ts.twist.linear.y = 0;
//   ts.twist.linear.z = 0;
//   vel_pub.publish(ts);
// }
// void publishAccelStamped(ros::Publisher accel_pub,ros::Time current_time,double angular_x,double angular_y,double angular_z, double linear_x, double linear_y, double linear_z)
// {
//   //geometry_msgs::Accel accel;
//   //twist.
//   geometry_msgs::AccelStamped as;
//   as.header.stamp = current_time;
//   as.accel.angular.x = angular_x;
//   as.accel.angular.y = angular_y;
//   as.accel.angular.z = angular_z;
//   as.accel.linear.x = linear_x;
//   as.accel.linear.y = linear_y;
//   as.accel.linear.z = linear_z;
//   accel_pub.publish(as);
// }

class gsofHandler : public rclcpp::Node
{
public:
  gsofHandler();
  ~gsofHandler();

private:
  void on_sentence(std_msgs::msg::ByteMultiArray::SharedPtr msg);
  void publish_tf(std::string child_frame_id, geometry_msgs::msg::PoseStamped p);

  rclcpp::Publisher<VehicleKinematicState>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_sentence_;

  // ros::Publisher pub_pose_;
  // ros::Publisher vel_pub_,acc_pub_;
  // ros::Publisher pub_week,pub_time;
  // tf::TransformBroadcaster br;
  // ros::Time current_time;
  gsofParser Parser; //paser for gsof
  geo_pos_conv geo;  //convert from lat-lon to x-y
  size_t count_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

gsofHandler::gsofHandler()
    : Node("gsof_parser"), count_(0)
{
  // Setup Tf Buffer with listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,
                                                              std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,
  //   std::shared_ptr<rclcpp::Node>(this, [](auto) {}), true);

  sub_sentence_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "gsof_sentence", 10, std::bind(&gsofHandler::on_sentence, this, _1));

  pub_kinematic_state_ = this->create_publisher<VehicleKinematicState>("vehicle_state", 10);
  pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

  geo.set_plane(32.1028, 35.2094); //tmp

  // sub_gsof_ = n.subscribe("gsof_sentence", 100, &gsofHandler::callbackFromGsofSentence, this);
  // pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);//gnss_pose
  // vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("gnss_velocity", 10);
  // acc_pub_ = n.advertise<geometry_msgs::AccelStamped>("gnss_acceleration", 10);
  // pub_week = n.advertise<std_msgs::UInt32>("gps_week", 10);
  // pub_time = n.advertise<std_msgs::UInt32>("gps_time", 10);
}
gsofHandler::~gsofHandler()
{
}

void gsofHandler::on_sentence(std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    // unsigned char * sentence = (unsigned char *)msg->data.c_str();
    // RCLCPP_WARN(this->get_logger(),"msg->data.length() %x  ",msg->data.length());
    // RCLCPP_WARN(this->get_logger(),"msg->data.size() %x  ",msg->data.size());

  // for(size_t i =0;i< msg->data.length();i++)
  // for(size_t i =0;i< 115;i++)
  //   RCLCPP_WARN(this->get_logger(),"%x  ", msg->data[i]);
  // RCLCPP_WARN(this->get_logger(),"on_sentence"); 

  // try
  // {
  

  // RCLCPP_WARN(this->get_logger(),"on_sentence1"); 
  
  auto current_time = this->now();//msg->header.stamp; //ros::Time::now();//
  InsFullNavigationInfo navInfo;
  unsigned char sentence[120];
  RCLCPP_WARN(this->get_logger()," msg->data.size() %u: ", msg->data.size()); 
  if(msg->data.size() != 115){
    return;
  }
  for (int i = 0; i < msg->data.size(); i++){
    sentence[i] = msg->data[i];
      }

  int complete_gsof = Parser.postGsofData(sentence, navInfo);//(unsigned char *)msg->data.c_str()
  // RCLCPP_WARN(this->get_logger(),"complete_gsof"); 
  //ROS_WARN_STREAM(" lat: "<<navInfo.Latitude<<" lon: "<<navInfo.Longitude<<" navInfo.Altitude: "<<navInfo.Altitude<<" complete_gsof: "<<complete_gsof);
    // RCLCPP_WARN(this->get_logger(),"nav: %f, %f, %f  ",navInfo.Latitude, navInfo.Longitude, navInfo.Altitude);
  // RCLCPP_WARN(this->get_logger(),"complete_gsof %d ",complete_gsof);

  // if (complete_gsof < 0) //if (complete_gsof == 1)//BUG
  // {
  // RCLCPP_ERROR(this->get_logger(),"complete_gsof %d ",complete_gsof);
    
  // }
    // RCLCPP_WARN(this->get_logger(),"current_time %d, %d ",current_time.sec,current_time.nanosec);
    // current_time.sec = navInfo.GPS_Week_Number*7*24*60*60;
    // current_time.nanosec = navInfo.GPS_Time;


    navInfo.Roll = deg2rad(navInfo.Roll);
    navInfo.Pitch = -deg2rad(navInfo.Pitch);
    navInfo.Heading = -deg2rad(navInfo.Heading) + M_PI / 2;
    navInfo.TrackAngle = -deg2rad(navInfo.TrackAngle);

    navInfo.AngularRate_x = deg2rad(navInfo.AngularRate_x);
    navInfo.AngularRate_y = -deg2rad(navInfo.AngularRate_y);
    navInfo.AngularRate_z = -deg2rad(navInfo.AngularRate_z);

    //navInfo.Longitudinal_acceleration =
    navInfo.Traverse_acceleration = -navInfo.Traverse_acceleration;
    navInfo.Down_acceleration = -navInfo.Down_acceleration;

    //ROS_WARN_STREAM(" lat: "<<navInfo.Latitude<<" lon: "<<navInfo.Longitude<<" navInfo.Altitude: "<<navInfo.Altitude);
    geo.set_llh(navInfo.Latitude, navInfo.Longitude, navInfo.Altitude);
  // RCLCPP_WARN(this->get_logger(),"set_llh"); 

    //ROS_WARN_STREAM("x: "<<geo.x()<<" y: "<<geo.y()<<" z: "<<geo.z());
    // publishPoseStamped(pub_pose_,current_time,geo.x(),geo.y(),geo.z(), navInfo.Roll, navInfo.Pitch, navInfo.Heading);
    // publishTF(br,current_time,geo.x(),geo.y(),geo.z(), navInfo.Roll, navInfo.Pitch, navInfo.Heading);
  

  //   if (!m_tf_buffer->canTransform("map", msg->header.frame_id, tf2::TimePointZero)) {
  //   return;
  // }



  // navInfo.Roll = 0.0, navInfo.Pitch = 0.0, navInfo.Heading = 0.0;
  // pose.header.stamp = this->now();
  
  // pose.pose.position.x = 1;
  // pose.pose.position.y = 0;
  // pose.pose.position.z = 0;
  //ROS_WARN_STREAM("x: "<<pose.pose.position.x<<" y: "<<pose.pose.position.y<<" z: "<<pose.pose.position.z);

  geometry_msgs::msg::TransformStamped tf;
  // tf2::Quaternion quat_tf;
  // quat_tf.setRPY(navInfo.Roll, navInfo.Pitch, navInfo.Heading);
  // tf2::convert(quat_tf, pose.pose.orientation);
  // pose.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(navInfo.Roll, navInfo.Pitch, navInfo.Heading);
  std::string odom_frame_id = "map"; //or map
  std::string gps_origin_frame_id = "gps_origin";
  // transform to fixed frame:
  // tf2::Duration timeout = tf2::durationFromSec(0.2);
  // tf2::Duration timeout = tf2::durationFromSec(10.0);
  // if (tf_buffer_->canTransform(m_odom_frame_id, pose.header.frame_id,
  //   tf2_ros::fromMsg(pose.header.stamp), timeout) )
  if (tf_buffer_->canTransform(odom_frame_id, gps_origin_frame_id, tf2::TimePointZero))

  // m_tf_buffer->canTransform("map", msg->header.frame_id, tf2::TimePointZero)
  {
    // auto pose_tansformed = tf_buffer_->transform(pose, m_odom_frame_id, timeout);

    // rclcpp::Time now = this->get_clock()->now();

    try
    {
      // needs to work without time_utils:
      // tf = tf_buffer_->lookupTransform(odom_frame_id, gps_origin_frame_id,
      //                                  time_utils::from_message(this->now()));
        tf = tf_buffer_->lookupTransform(odom_frame_id, gps_origin_frame_id, this->now());
    }
    catch (const tf2::ExtrapolationException &)
    {
      // TODO(mitsudome-r): currently falls back to retrive newest
      // transform available for availability,
      // Do validation in the future
      tf = tf_buffer_->lookupTransform(odom_frame_id, gps_origin_frame_id, tf2::TimePointZero);
      return;
    }
  }
  else
  {
    RCLCPP_WARN(
        this->get_logger(), "on_sentence cannot transform %s to %s",
        gps_origin_frame_id.c_str(), odom_frame_id.c_str());
        return;
  }
  RCLCPP_WARN(this->get_logger(),"canTransform"); 


  tf2::Quaternion quat_tf;
  geometry_msgs::msg::Quaternion q;
  quat_tf.setRPY(navInfo.Roll, navInfo.Pitch, navInfo.Heading);
  // RCLCPP_WARN(this->get_logger(),"ang: %f, %f, %f  ",navInfo.Roll, navInfo.Pitch, navInfo.Heading);

  // quat_tf.setRPY(0, 0, 0);
  // tf2::convert(quat_tf, q);//works just on foxy
  q = tf2::toMsg(quat_tf);//for dashing

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = gps_origin_frame_id;
  pose.header.stamp = current_time;
  pose.pose.position.x = geo.y();
  pose.pose.position.y = geo.x();
  pose.pose.position.z = geo.z();
  // RCLCPP_WARN(this->get_logger(),"nav: %f, %f, %f  ",pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  // pose.pose.position.x = 0.0;
  // pose.pose.position.y = 0.0;
  // pose.pose.position.z = 0.0;
  pose.pose.orientation = q;
  RCLCPP_WARN(this->get_logger(),"pose"); 


  geometry_msgs::msg::PoseStamped transformed_pose;
  tf2::doTransform(pose, transformed_pose, tf);

  VehicleKinematicState state;
  state.header.frame_id = gps_origin_frame_id;
  state.header.stamp = current_time;
  state.state.x = transformed_pose.pose.position.x;
  state.state.y = transformed_pose.pose.position.y;
  const auto inv_mag = 1.0 / std::sqrt((transformed_pose.pose.orientation.z * transformed_pose.pose.orientation.z) + (transformed_pose.pose.orientation.w * transformed_pose.pose.orientation.w));;
  state.state.heading.real = static_cast<decltype(state.state.heading.real)>(transformed_pose.pose.orientation.w * inv_mag);
  state.state.heading.imag = static_cast<decltype(state.state.heading.imag)>(transformed_pose.pose.orientation.z * inv_mag);
  state.state.longitudinal_velocity_mps = navInfo.TotalSpeed;

  // RCLCPP_WARN(this->get_logger(),"state"); 

  
  // RCLCPP_WARN(this->get_logger(),"nav: %f  ",navInfo.TotalSpeed);


  // motion::motion_common::doTransform(state, transformed_state, tf);

  // tf2::Quaternion transformed_quat_tf;

  // geometry_msgs::msg::Quaternion transformed_q;
  // tf2::doTransform(q, transformed_q, tf);
  // motion::motion_common::doTransform(quat_tf, transformed_quat_tf, tf);

  // transformed_q.x =0;
  // transformed_q.y =0;
  // transformed_q.z =transformed_state.state.heading.imag*mag ;
  // transformed_q.w =transformed_state.state.heading.real*mag ;

  // publish state:

  pub_kinematic_state_->publish(state);
  publish_tf("base_link",transformed_pose);
  // publishPoseStamped(pub_pose_,current_time,geo.x(),geo.y(),geo.z(), navInfo.Roll, navInfo.Pitch, navInfo.Heading);
  // publishTF(br,current_time,geo.x(),geo.y(),geo.z(), navInfo.Roll, navInfo.Pitch, navInfo.Heading);
  // //ROS_WARN_STREAM("roll: "<<navInfo.Roll<<" pitch: "<< navInfo.Pitch<<" yaw: "<<navInfo.Heading);

  // publishVelStamped(vel_pub_,current_time,navInfo.AngularRate_x,navInfo.AngularRate_y,navInfo.AngularRate_z, navInfo.NorthVelocity, navInfo.EastVelocity, navInfo.DownVelocity, navInfo.TotalSpeed);
  // publishAccelStamped(acc_pub_,current_time,0,0,0, navInfo.Longitudinal_acceleration, navInfo.Traverse_acceleration, navInfo.Down_acceleration);
  // publishTime(pub_week,pub_time,navInfo.GPS_Week_Number,navInfo.GPS_Time);
  //publish all;
  // }
  // else if (complete_gsof == -1)
  // RCLCPP_ERROR(this->get_logger(),"Error - parsing gsof");
  // RCLCPP_WARN(this->get_logger(),"publish"); 

    // }
  // catch(const std::exception& e)
  // {
  //   std::cerr << e.what() << '\n';
  // }
}

void gsofHandler::publish_tf(std::string child_frame_id, geometry_msgs::msg::PoseStamped p)
{
  geometry_msgs::msg::TransformStamped tf{};
  tf.header = p.header;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = p.pose.position.x;
  tf.transform.translation.y = p.pose.position.y;
  tf.transform.translation.z =0.0;// p.pose.position.z;
  tf.transform.rotation = p.pose.orientation;

  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));
  pub_tf_->publish(tf_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gsofHandler>());
  rclcpp::shutdown();
  return 0;
}