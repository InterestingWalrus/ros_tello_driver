#include <ros/ros.h>
#include <thread>
#include  <iostream>
#include <mutex>
#include <boost/asio.hpp>
#include "std_msgs/String.h"

#include "ros_tello_driver/udp_client.h"
#include "ros_tello_driver/udp_server.h"

#define IP_ADDRESS "192.168.10.1"
#define CMD_UDP_PORT "8889"

#define SPIN_RATE 100 // Change this later to ROS::Rate

using boost::asio::ip::udp;

class TelloDriver
{

    public:
    TelloDriver();
    ~TelloDriver();
    //void stateCallback(const std_msgs::String::ConstPtr& msg); // change to callbacks later. 
    void process_state_packet(std::string state_);
    void process_video_packets(std::string video_);
    void activate_drone();
    void keep_drone_alive(); // Tello will autoland if nothing is heard for 15 seconds. We want to prevent that here.
    void run();




    private:
    bool connected = false;
    bool  streaming = false;

    ros::Time state_recv_time;
    ros::Time video_recv_time;

    boost::asio::io_service io_service_;

    // Threads for mutexes;
    std::mutex mutex_;
    std::thread state_thread;
    std::thread vid_stream_thread;
    ros::NodeHandle nh;

    udp::endpoint state_endpoint{udp::v4(), 8890};
    udp::endpoint video_endpoint{udp::v4(), 11111};

     UDPClient udp_client{io_service_, IP_ADDRESS, CMD_UDP_PORT};
    // udp_server state_server{io_service_, state_endpoint};
    // udp_server video_server{io_service_, video_endpoint};

    udp_server video_server{io_service_, 11111};
    udp_server state_server{io_service_, 8890};

 
};
