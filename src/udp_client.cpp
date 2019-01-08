
#include "ros_tello_driver/udp_client.h"
#include "ros_tello_driver/udp_server.h"



UDPClient::UDPClient(boost::asio::io_service& io_service, std::string host, std::string port):     
                     io_service(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0))             
{

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), host, port);
    udp::resolver::iterator iter = resolver.resolve(query);
    remote_endpoint = *iter;

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
  std::cout << "Length of received message" << len << std::endl;
  std::cout << "Response: " << result << std::endl;
  std::cout.write(recv_buffer.data(), len);
}

int main (int argc, char ** argv)
{

  ros::init(argc, argv, "tello_udp_client ");

  std::thread command_thread();

  
  // try
  //   {

  //     boost::asio::io_service server_io_service;
  //     boost::asio::io_service io_service;
  //     udp_server server(server_io_service, 8890);

  //     io_service.run();
  //     UDPClient udp_client(io_service, IP_ADDRESS, CMD_UDP_PORT);
  
  //       std::string cmd;

  //      while(cmd != "quit")
  //      {

  //         std::cout << "Enter command: " << std::endl;      

  //         std::cin >> cmd ; 
  //         udp_client.send(cmd);
  //         udp_client.receive();


  //     //  }


      


  //   }

  //   catch(std::exception& e)
  //   {
  //     std::cerr << "Error occured" << e.what() << std::endl;
  //   }
    
  

  ros::spin();

  
   

    return 0;
}