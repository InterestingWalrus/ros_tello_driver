#include "ros_tello_driver/udp_server.h"

udp_server::udp_server(boost::asio::io_service& io_service, int port_number): socket_(io_service, udp::endpoint(udp::v4(), port_number))
{

  // remote_endpoint = udp::endpoint(address, 8890);
    std::string ip_address = socket_.local_endpoint().address().to_string();
  

    std::cout << "Listening on :" << ip_address << ":" << port_number << std::endl; 
    start_receive();
}

void udp_server::start_receive()
{

    std::cout<< "receiving" << std::endl;

   socket_.async_receive_from( boost::asio::buffer(sample_buffer), remote_endpoint, 
                                 boost::bind(&udp_server::handle_receive, this, boost::asio::placeholders::error,
                                 boost::asio::placeholders::bytes_transferred));
}

void udp_server::handle_receive(const boost::system::error_code& error, std::size_t)
{
    if(!error || error ==boost::asio::error::message_size)
    {
        double  time_now = 50;

        std::cout << time_now << std::endl;

       // std::string time_to_string = time_now;

        boost::shared_ptr<std::string> message( new std::string ("Hi"));

        socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint, 
         boost::bind(&udp_server::handle_send, this, message,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));

            start_receive();
    }
}

void udp_server::handle_send(boost::shared_ptr<std::string> , const boost::system::error_code& ,std::size_t )
{

}


std::string udp_server::get_Recv_Buffer()
{

    return sample_buffer;

}
