#ifndef UDP_CLIENT
#define UDP_CLIENT

#include <iostream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <ros/ros.h>

#define IP_ADDRESS "192.168.10.1"
#define CMD_UDP_PORT "8889"
#define STATE_UDP_PORT "8890"
#define VIDEO_STREAM_PORT "11111"


using boost::asio::ip::udp;
using boost::asio::ip::address;


using namespace std;

class UDPClient
{
  
  public:
        UDPClient(boost::asio::io_service& io_service, std::string host, std::string port);
        ~UDPClient();
        void send( std::string data );
         void send_buf(boost::array<unsigned char, 1024> buf);
        void receive();
        void runUDPClient();





  private:

  boost::asio::io_service& io_service;   // Manages IO
  udp::socket  socket_;  // socket
  udp::endpoint remote_endpoint; 
  udp::endpoint sender_endpoint;
  boost::array< char, 1024> recv_buffer ;
  boost::array< char, 1> send_buffer = {{0}} ;

 

};

#endif
