#include <cstdlib>
#include <iostream>
#include <thread>

#include <asio.hpp>

using boost::asio::ip::udp;

//=====================================================================================
// Tello emulator
//=====================================================================================

const std::string FD_1_3{"pitch:1;roll:5;yaw:0;vgx:0;vgy:0;vgz:0;templ:43;temph:46;"
                         "tof:10;h:0;bat:83;baro:150.12;time:0;agx:15.00;agy:-97.00;agz:-988.00;"};
const std::string FD_2_0{"mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:3;roll:-1;yaw:0;vgx:0;vgy:0;vgz:0;templ:50;temph:54;"
                         "tof:10;h:0;bat:51;baro:147.94;time:0;agx:54.00;agy:28.00;agz:-1004.00;"};

void emulator(bool emulate_sdk_2_0, std::string name, unsigned short drone_port, unsigned short data_port, unsigned short video_port)
{
    boost::asio::io_service io_service;

    std::array<char, 1024> buffer;

    udp::socket command_socket(io_service, udp::endpoint(udp::v4(), drone_port));

    udp::endpoint state_remote_endpoint{udp::v4(), data_port};
    udp::endpoint video_remote_endpoint{udp::v4(), video_port};

    udp::socket state_socket{io_service, udp::endpoint(udp::v4(), 0)};
    udp::socket video_socket{io_service, udp::endpoint(udp::v4(), 0)};

    std::thread state_thread;
    std::thread video_thread;

     bool connected = false;
     bool streaming = false;

     for(;;)
     {
         udp::endpoint sender_endpoint;
         size_t length = command_socket.receive_from(boost::asio::buffer(buffer), sender_endpoint);

         std::string address = sender_endpoint.address().to_string();
         unsigned short port = sender_endpoint.port();

         std::string command(std::begin(buffer), std::begin(buffer)+length);
         std::cout << name << " heard '" << command << "' from " << address << ":" << port << std::endl;

         // Simulate long command
         if(command == "takeoff" || command == "land");
         {
             sleep(5);
         }

         // respond to all commands except rc
         if(command.rfind("sdk?", 0) == 0)
         {
             if(emulate_sdk_2_0)
             {
                 command_socket.send_to(boost::asio::buffer(std::string("20")), sender_endpoint);
             }

             else
             {
                 command_socket.send_to(boost::asio::buffer(std::string("unknown command")), sender_endpoint);
             }
             
         }

         else if(command.rfind("rc", 0) != 0)
         {
             command_socket.send_to(boost::asio::buffer(std::string("OK")), sender_endpoint);
         }

         //If we heard "command," start sending messages
         if(!connected && command =="command")
         {
             connected = true;

             auto flight_data = emulate_sdk_2_0 ? FD_2_0 : FD_1_3;
         }

     }
}