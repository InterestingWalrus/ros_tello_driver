#include <ros/ros.h>
#include <thread>
#include  <iostream>
#include <mutex>
#include <asio.hpp>

#include "ros_tello_driver/udp_client.h"
#include "ros_tello_driver/udp_server.h"

#define IP_ADDRESS "192.168.10.1"
#define CMD_UDP_PORT "8889"

using asio::ip::udp;

class Tello_Driver
{

    public:
    Tello_Driver();



    private:
    bool connected = false;
    bool  streaming = false;

    boost::asio::io_service io_service_;

    // Threads for mutexes;
    std::mutex mutex_;
    std::thread state_thread;
    std::thread vid_stream_thread;

    udp::endpoint state_endpoint(udp::v4(), 8890);
    udp::endpoint video_endpoint(udp::v4(), 11111);

    UDPClient udp_client(io_service_, IP_ADDRESS, CMD_UDP_PORT);
    udp_server state_server(io_service_, state_endpoint);
    udp_server video_server(io_service_, video_endpoint);

    



    

};
