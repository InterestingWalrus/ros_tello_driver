#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>

using boost::asio::ip::udp;
using boost::asio::ip::address;
namespace io = boost::iostreams;

#define IP_ADDRESS "192.168.10.1"
#define STATE_UDP_PORT "8890"



class udp_server
{
    public:
     udp_server(boost::asio::io_service& io_service,  udp::endpoint server_endpoint);
      std::string get_Recv_Buffer();

     private:
     void start_receive();
     void handle_receive(const boost::system::error_code& error, std::size_t bytes_tx);

     udp::socket socket_;
     udp::endpoint remote_endpoint;
     boost::array<char, 1024> recv_buffer_;
     std::string sample_buffer;

     static const size_t max_length_ = 1024;
     size_t len;


};