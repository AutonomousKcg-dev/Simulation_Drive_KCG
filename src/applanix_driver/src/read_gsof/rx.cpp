/**
Software License Agreement (BSD)

\file      rx.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <read_gsof/rx.hpp>


// #include <poll.h>
// #include <sstream>
// #include <sys/ioctl.h>
// #include <boost/thread.hpp>
// #include <boost/algorithm/string.hpp>

#include <string>

#define RX_INITIAL_LENGTH 40
#define RX_SUCCESSIVE_LENGTH 8

int complete_gsof(char* buffer,int length)
{
  int data = *buffer;//skip stat
  printf("start: %d",data);
  if(data != 0x02)
    return -1;
  data = *(buffer+2);
  printf("type: %d",data);
  if(data != 0x40)
    return -1;

  data = *(buffer+3);//lenght
  printf("lenght in message: %d length readed %d",data, length);
  if (length < data+6)
    return 0;
  if(buffer[length-1] == 0x03)
    return 1;
  else
    return -1;
  
}
std::string convertToString(char* a, int size) 
{ 
    int i; 
    std::string s = ""; 
    for (i = 0; i < size; i++) { 
        s = s + a[i]; 
    } 
    return s; 
} 
// void _handle_sentence(rclcpp::Publisher<additional_msgs::msg::Sentence> & publisher, ros::Time& stamp, char* sentence,int buffer_len, const char* frame_id)
// {
//   //ROS_WARN("sentence2: %s", sentence);
//   //ROS_DEBUG("Sentence RX: %s", sentence);


//   static nmea_msgs::Sentence sentence_msg;
//   sentence_msg.sentence = convertToString(sentence,buffer_len);
//   sentence_msg.header.stamp = stamp;
//   sentence_msg.header.frame_id = frame_id;
//   // printf("sentence: ");
//   // for(int i =0;i<sentence_msg.sentence.length();i++)
//   //   printf("%x  ", sentence_msg.sentence[i]);
//   // printf("\n");
//   //ROS_WARN_STREAM(ros::Time::now());

//   publisher.publish(sentence_msg);
// }

