

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

// #include <additional_msgs/msg/sentence.h> // nmea sentence msg
// #include <autoware_auto_msgs/msg/sentence.hpp> // nmea sentence msg
#include <autoware_auto_msgs/msg/complex32.hpp>
using Complex32 = autoware_auto_msgs::msg::Complex32;

// #include <vrxperience_msgs/msg/gps.hpp>


#include <read_gsof/rx.hpp>

#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
// using Sentence = additional_msgs::msg::Sentence;
// using Sentence = autoware_auto_msgs::msg::Sentence;


class SocketNode : public rclcpp::Node
{
public:
  SocketNode()
  : Node("gsof_socket"), count_(0)
  {
    //publishers:
    pub_gsof_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("gsof_sentence", 10);//std_msgs::msg::String
  
    int listener_fd = socket(AF_INET,SOCK_DGRAM, 0);// SOCK_STREAM
    if (listener_fd < 0){
      RCLCPP_FATAL(this->get_logger(), "ERROR opening socket");
      rclcpp::shutdown();
    }
    RCLCPP_WARN(this->get_logger(), "listener_fd %d",listener_fd);


    int port = 5005;
    // n_local.param("port", port, 5005);

    std::string frame_id = "navsat";

    /* Initialize socket structure */
    struct sockaddr_in serv_addr;//, cli_addr;
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;//INADDR_BROADCAST;//inet_pton("192.168.1.100");//INADDR_ANY;
    serv_addr.sin_port = htons( __uint16_t(port));

    /* Now bind the host address using bind() call.*/
    int previous_success = 1;
    while (1){
      if (bind(listener_fd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0)
      {
        if (previous_success){
          RCLCPP_ERROR(this->get_logger(),"Unable to bind socket. Is port %d in use? Retrying every 1s.", port);
          previous_success = 0;
        }
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      }
      else{
        break;
      }
    }


    // unsigned int clilen = sizeof(cli_addr);
    char buf[2048];//unsigned 
    RCLCPP_WARN(this->get_logger(), "Now listening for connections on port %d.", port);

    RCLCPP_INFO(this->get_logger(),"Now listening for connections on port %d.", port);
    // struct pollfd pollfds[] = { { listener_fd, POLLIN, 0 } };

    struct sockaddr_in remaddr;
    socklen_t addlen = sizeof(remaddr);
    auto now = this->now();
;

  while (rclcpp::ok()){
    //int retval = poll(pollfds, 1, 500);
    ssize_t retval = recvfrom(listener_fd,buf,2048,0,(struct sockaddr *)&remaddr,&addlen);
    // now = this->now(); //rclcpp::Clock::now();
    //ROS_WARN_STREAM("received: "<<retval);

      // publish sentence:
    // static Sentence sentence_msg;
    // sentence_msg.sentence = convertToString(buf,int(retval));
    // sentence_msg.header.stamp = now;//to msg?
    // sentence_msg.header.frame_id = frame_id;
    // RCLCPP_WARN(this->get_logger(), "received: %d.", retval);

    // std_msgs::msg::String sentence_msg;
    std_msgs::msg::ByteMultiArray sentence_msg;
    for(int i =0;i<retval;i++)
       sentence_msg.data.push_back(buf[i]);

    // sentence_msg.data = convertToString(buf,int(retval));

  //  RCLCPP_WARN(this->get_logger(),"data: %d",sentence_msg.data.length());
    // for(size_t i =0;i<sentence_msg.data.size();i++)
    //    RCLCPP_WARN(this->get_logger(),"%x  ", sentence_msg.data[i]);
    // printf("\n");
    //ROS_WARN_STREAM(ros::Time::now());

    pub_gsof_->publish(sentence_msg);

    // _handle_sentence(pub, now, buf,retval, frame_id.c_str());
  }



  }

private:
  

  //publishers:
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_gsof_;
  size_t count_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SocketNode>());
  rclcpp::shutdown();
  return 0;
}
