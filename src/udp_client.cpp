
#include "ros_tello_driver/udp_client.h"



UDPClient::UDPClient(boost::asio::io_service& io_service, 	 std::string host,   std::string port):
                     io_service(io_service), socket(io_service, udp::endpoint(udp::v4(), 0))
{

  
  udp::resolver resolver(io_service);
  udp::resolver::query query(udp::v4(), host, port);
  remote_endpoint = *resolver.resolve(query);


  boost::array<char, 1024> recv_buffer = {0};
   

}

UDPClient::~UDPClient()
{

    socket.close();


}

void UDPClient::send(const unsigned char data [])
{

    socket.send_to(boost::asio::buffer(data, sizeof(data)), remote_endpoint);

}

void UDPClient::receive()
{
    size_t len = socket.receive_from(boost::asio::buffer(recv_buffer), sender_endpoint);

    std::cout.write(recv_buffer.data(), len);
}

int main (int argc, char ** argv)
{

ros::init(argc, argv, "tello_udp_client ");
  boost::asio::io_service io_service;

  UDPClient client(io_service, IPADDRESS, UDP_PORT);

  try
  {

      const char connect_command[] =  "conn_req:\x96\x17";


      unsigned char connection_req [sizeof(connect_command)];

      std::copy(connect_command, connect_command + sizeof(connect_command), connection_req);

     client.send(connection_req);
  }  

  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  ros::spin();



    return 0;
}