#ifndef UDP_CLIENT
#define UDP_CLIENT

#include <iostream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <ros/ros.h>

#define IPADDRESS "192.168.0.1"
#define UDP_PORT "8889"


using boost::asio::ip::udp;
using boost::asio::ip::address;


using namespace std;

class UDPClient
{
  
  public:
        UDPClient(boost::asio::io_service& io_service, 	std::string host,   std::string port);
        ~UDPClient();
        void send( unsigned char data [] );
        void receive();
       // void receive(const boost::system::error_code& error, size_t bytes_transferred);
        //void reciever();
      //  void wait();




  private:

  boost::asio::io_service& io_service;
  udp::socket  socket;
  udp::endpoint remote_endpoint;
  udp::endpoint sender_endpoint;
  boost::array<char, 1024> recv_buffer ;
 

};

#endif
