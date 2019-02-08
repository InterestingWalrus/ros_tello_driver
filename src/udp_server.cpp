#include "ros_tello_driver/udp_server.h"

udp_server::udp_server(boost::asio::io_service& io_service,  udp::endpoint server_endpoint): socket_(io_service, udp::v4()),
   remote_endpoint()
{

    boost::asio::socket_base::reuse_address option(true);
    socket_.set_option(option);
    socket_.bind(server_endpoint);

    std::string ip_address = socket_.local_endpoint().address().to_string();
   // std::string port_number = 
  
    //std::cout << "Listening on " << ip_address << ":" << port_number << std::endl; 
    start_receive();
}

void udp_server::start_receive()
{

    std::cout<< "receiving" << std::endl;

    socket_.async_receive_from( boost::asio::buffer(recv_buffer_), 
                                remote_endpoint, 
                                boost::bind(&udp_server::handle_receive,
                                            this, boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));    
    
}

void udp_server::handle_receive(const boost::system::error_code& error, std::size_t received_bytes)
{
    std::cout << "Ever reach here" << std::endl;
    if(!error || error == boost::asio::error::message_size)
    {
       
         std::cout << "RECV bytes" << received_bytes << std::endl;

         start_receive();
    }
}

std::string udp_server::get_Recv_Buffer()
{
    std::string result;

   

  // std::cout << "Answer: " << sample_buffer << std::endl;


}
