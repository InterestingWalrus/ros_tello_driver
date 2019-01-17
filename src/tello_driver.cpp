#include "ros_tello_driver/tello_driver.h"

TelloDriver::TelloDriver()
{

    UDPClient udp_client(io_service_, IP_ADDRESS, CMD_UDP_PORT);
    udp_server state_server(io_service_, state_endpoint);
    udp_server video_server(io_service_, video_endpoint);

    state_thread = std::thread(
    [this]()
    {

        for (;;)
        {
            size_t r = state_server.get_Recv_Buffer();
            mutex_.lock();
            process_state_packet(r);
            mutex_.unlock();

        }

    });

    video_thread = std::thread(
    [this]()
    {

        for (;;)
        {
            size_t r = video_server.get_Recv_Buffer();
            mutex_.lock();
            process_video_packets(r);
            mutex.unlock();
        }

    });
    
}

TelloDriver::~Tello_Driver()
{



}

void TelloDriver::process_state_packet(size_t state_)
{

    if(!connected)
    {
        //ROS_INFO("Receiving state %s", state_);
        std::cout << "receiving state: " << state_ << std::endl;
        connected = true;
    }

}

void TelloDriver::process_video_packets(size_t video_)
{

    if(!streaming)
    {
        //ROS_INFO("Receiving state %s", state_);
        std::cout << "receiving state: " << video_ << std::endl;
        streaming = true;
    }

}

void TelloDriver::activate_drone()
{
    if(!connected)
    {
        std::cout<< "Activating SDK" << std::endl;
        udp_client.send("command");
        
        // receive response from drone
        udp_client.receive();

    }

    if(connected && !streaming )
    {
        std::cout<< "Activating Video Transmission" << std::endl;
        udp_client.send("streamon");
        
        // receive response from drone
        udp_client.receive();

    }

    
}

//void TelloDriver::stateCallback()

int main (int argc, char ** argv)
{

  ros::init(argc, argv, "tello_udp_client ");

 
    
  

  ros::spin();

  
   

    return 0;
}
