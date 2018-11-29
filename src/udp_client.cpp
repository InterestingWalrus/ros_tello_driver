
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

void UDPClient::send( unsigned char data [])
{

    int data_length = (unsigned) strlen((char*)data);
    ROS_INFO("Data Length = % d", data_length);
    ROS_INFO("Data: %s", data);

    socket.send_to(boost::asio::buffer(data, data_length), remote_endpoint);

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

       int cmd_one = std::hex << 96;
       int cmd_two = std::hex << 17;
       char connect_command[] =  "conn_req:";

      int command_length = (unsigned) strlen(connect_command);
     
      ROS_INFO("Size of command: %d", command_length);


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