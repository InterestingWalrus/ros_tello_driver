#ifndef TELLO_SOCKET_H
#define TELLO_SOCKET_H

#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <thread>
#include  <iostream>
#include <boost/asio.hpp>
//#include "ros_tello_driver/tello_driver.h" add to cpp

using boost::asio::ip::udp;

class TelloDriver;

class TelloSocket
{

    protected:

        void listen();
        virtual void process_packet(size_t packet);

        TelloDriver *tello_driver;           // Pointer to driver
        boost::asio::io_service io_service_; // Manages IO for the socket
        udp::socket socket_;                 // socket to open
        std::thread thread_;                  // Thread for each socket; command, video and state will run individual threads
        std::mutex mutex_;                   // Blocks resource when in use
        bool rx = false;              // checks if packets are being received on the socket
        ros::Time   recv_time_;              // Latest receive time
        std::vector<unsigned char> buffer_ ;   // Packet buffer to store data

    public:
        TelloSocket(TelloDriver *driver, unsigned short port): tello_driver(driver), socket_(io_service_, udp::endpoint(udp::v4(), port))
        {

        }
    
        bool receiving();
        ros::Time recv_time();        // Latest msg timestamp
        virtual void timeout();


};



#endif