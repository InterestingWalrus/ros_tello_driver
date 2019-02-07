#include "ros_tello_driver/tello_driver.h"

TelloDriver::TelloDriver()
{

    UDPClient udp_client(io_service_, IP_ADDRESS, CMD_UDP_PORT);
   udp_server state_server(io_service_, 8890);
   udp_server video_server(io_service_, 11111);

    state_thread = std::thread(
    [&]()
    {

        for (;;)
        {
            string r = state_server.get_Recv_Buffer();
            mutex_.lock();
            process_state_packet(r);
            mutex_.unlock();

        }

    });

    vid_stream_thread = std::thread(
    [&]()
    {

        for (;;)
        {
            string r = video_server.get_Recv_Buffer();
            mutex_.lock();
            process_video_packets(r);
            mutex_.unlock();
        }

    });
    
}

void TelloDriver::run()
{
    mutex_.lock();
    static int counter = 0;
    counter ++;
    if(counter % SPIN_RATE == 0)
    {
        activate_drone();
    }

    if (counter % (5 * SPIN_RATE) == 0)
    {
        keep_drone_alive();
    }

    mutex_.unlock();

}

TelloDriver::~TelloDriver()
{



}

void TelloDriver::process_state_packet(std::string state_)
{

    state_recv_time = ros::Time::now();

    if(!connected)
    {
        //ROS_INFO("Receiving state %s", state_);
        std::cout << "receiving state: " << state_ << std::endl;
        connected = true;
    }

}

void TelloDriver::process_video_packets(std::string video_)
{
    video_recv_time = ros::Time::now();

    if(!streaming)
    {
        //ROS_INFO("Receiving state %s", state_);
        std::cout << "receiving state: " << video_ << std::endl;
        streaming = true;
    }

}

void TelloDriver::activate_drone()
{
    if(connected && ros::Time::now() - state_recv_time < ros::Duration(10))
    {

        ROS_ERROR("Drone no longer connected");
         connected = false;

        // Nothing receicved for 10 seconds, 
        // Assume drone isn't connected anymore

    }

     if(connected && ros::Time::now() - state_recv_time < ros::Duration(10))
    {
         // Nothing receicved for 10 seconds, 
        // Assume drone isn't streaming anymore

              ROS_ERROR("Drone no longer streaming");
            streaming  = false;
    }


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

void TelloDriver::keep_drone_alive()
{
    if(connected && streaming)
    {
        ROS_INFO("Keep drone in the air");
        udp_client.send("command");
    }
}

//void TelloDriver::stateCallback()

int main (int argc, char ** argv)
{

  ros::init(argc, argv, "tello_udp_client ");

   TelloDriver telloDriver;

   ros::Rate loop_rate(SPIN_RATE);

   while(ros::ok())
   {
       ros::spinOnce();

       loop_rate.sleep();
   }

 

    return 0;
}
