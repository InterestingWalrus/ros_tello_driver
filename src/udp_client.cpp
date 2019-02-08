
#include "ros_tello_driver/udp_client.h"
#include "ros_tello_driver/udp_server.h"



UDPClient::UDPClient(boost::asio::io_service& io_service, std::string host, std::string port):     
                     io_service(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0))             
{

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), host, port);
    udp::resolver::iterator iter = resolver.resolve(query);
    remote_endpoint = *iter;

    ROS_INFO("Called UDP CLient");

}

UDPClient::~UDPClient()
{

    std::cout << "Calling UDP client destructor" << std::endl;
    socket_.close();

}

void UDPClient::send( std::string data)
{
   
   socket_.send_to(boost::asio::buffer(data), remote_endpoint);

}

void UDPClient::send_buf(boost::array<unsigned char, 1024> buf)
{

  ROS_INFO("HERE");
   
   socket_.send_to(boost::asio::buffer(buf, 1024), remote_endpoint);

}


void UDPClient::runUDPClient()
{
  // try
  // {

  //   boost::asio::io_service;
  //   UDPClient udp_client(io_service, IP_ADDRESS, CMD_UDP_PORT);
  //   udp_client.send("command");
  //   udp_client.receive();


  // }

  // catch(std::exception& e)
  // {
  //   std::cerr << "Error occured" << e.what() << std::endl;
  // }
}

void UDPClient::receive()
{
  
 
  size_t len = socket_.receive_from(boost::asio::buffer(recv_buffer), sender_endpoint);

   std::string result;


  std::copy(recv_buffer.begin(), recv_buffer.begin()+ len , std::back_inserter(result) );
  
  std::cout<<"Drone Response  to command "<< result << std::endl;

}

