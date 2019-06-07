#include "ros_tello_driver/tello_driver.h"

// TelloDriver::TelloDriver()
// {

//     state_thread = std::thread(
//     [&]()
//     {

//         for (;;)
//         {
//             //string r = state_server.get_Recv_Buffer();
//             size_t r = state_socket.receive(boost::asio::buffer(state_buffer_, max_length_));
//             mutex_.lock();
//             process_state_packet(r);
//             mutex_.unlock();

//         }

//     });

//     vid_stream_thread = std::thread(
//     [&]()
//     {

//         for (;;)
//         {
//            // string r = video_server.get_Recv_Buffer();
//             size_t r = video_socket.receive(boost::asio::buffer(video_buffer_, max_length_));
//             mutex_.lock();
//             process_video_packets(r);
//             mutex_.unlock();
//         }

//     });
    
// }

// void TelloDriver::run()
// {
//     mutex_.lock();
//     static int counter = 0;
//     counter ++;
//     if(counter % SPIN_RATE == 0)
//     {
//         activate_drone();
//     }

//     if (counter % (5 * SPIN_RATE) == 0)
//     {
//         keep_drone_alive();
//     }

//     mutex_.unlock();

// }

// TelloDriver::~TelloDriver()
// {



// }

// void TelloDriver::process_state_packet(size_t state_)
// {

//     state_recv_time = ros::Time::now();

//     if(!connected)
//     {
//         //ROS_INFO("Receiving state %s", state_);
//         std::cout << "receiving drone state: " << state_ << std::endl;
//         connected = true;
//         ros::Duration(0.1).sleep();

//     }

// }

// void TelloDriver::process_video_packets(size_t video_)
// {
//     video_recv_time = ros::Time::now();

//     if(!streaming)
//     {
//         //ROS_INFO("Receiving state %s", state_);
//         std::cout << "receiving  video state: " << video_ << std::endl;
//         streaming = true;
//         ros::Duration(0.1).sleep();
//     }

// }

// void TelloDriver::activate_drone()
// {
//     if(connected && ros::Time::now() - state_recv_time > ros::Duration(5,0))
//     {

//          ROS_ERROR("Drone no longer connected");
//          connected = false;

//         // Nothing receicved for 10 seconds, 
//         // Assume drone isn't connected anymore

//     }

//      if(connected && ros::Time::now() - state_recv_time > ros::Duration(5,0))
//     {
//          // Nothing receicved for 10 seconds, 
//         // Assume drone isn't streaming anymore

//            ROS_ERROR("Drone no longer streaming");
//             streaming  = false;
//     }


//     if(!connected)
//     {
//         std::cout<< "Activating SDK" << std::endl;
//         udp_client.send("command");
//          //std::cout<< "Here?" << std::endl;
        
//         // receive response from drone
//         //udp_client.receive();

//     }

//     if(connected && !streaming )
//     {
//         std::cout<< "Activating Video Transmission" << std::endl;
//         udp_client.send("streamon");
        
//         // receive response from drone
//        // udp_client.receive();

//     }

    
// }

// void TelloDriver::keep_drone_alive()
// {
//     if(connected && streaming)
//     {
//         ROS_INFO("Keep drone in the air");

//         unsigned int arr[] = { 0xCC, 0x60, 0x00, 0x27, 0x68, 0x55, 0x00, 0xE5, 0x01, 0x00, 0xBA, 0xC7 };

//         boost::array<unsigned char, 1024> buf = { 0xCC, 0x58, 0x00, 0x7C, 0x68, 0x54, 0x00, 0xE4, 0x01, 0xC2, 0x16 };
        
//         //char buf[200];

//         // io::stream<io::array_sink> as(buf);

//         // as << boost::format("%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X")
//         // % arr[0]
//         // % arr[1]
//         // % arr[2]
//         // % arr[3]
//         // % arr[4]
//         // % arr[5]
//         // % arr[6]
//         // % arr[7]
//         // % arr[8]
//         // % arr[9]
//         // % arr[10]
//         //  % arr[10];

//         //  std::cout.write(buf, as.tellp());
//        // std::cout.write(buf.data(), 1024);


//         udp_client.send("land");
//        // udp_client.send_buf(buf);

//        // udp_client.receive();

//     }
// }

// //void TelloDriver::stateCallback()

int main (int argc, char ** argv)
{

//   ros::init(argc, argv, "tello_udp_client ");

//    TelloDriver telloDriver;

//    ros::Rate loop_rate(SPIN_RATE);

//    while(ros::ok())
//    {

//        telloDriver.run();
//        ros::spinOnce();

//        loop_rate.sleep();
//    }

 

    return 0;
}

