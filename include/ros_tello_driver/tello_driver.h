#include <ros/ros.h>
#include <thread>
#include  <iostream>
#include <mutex>
#include <boost/asio.hpp>
#include "std_msgs/String.h"

#include <ros_tello_driver/FlightTelemetry.h>
#include <ros_tello_driver/TelloAction.h>
#include <ros_tello_driver/TelloResponse.h>
//#include <ros_tello_driver/


#define IP_ADDRESS "192.168.10.1"
#define CMD_UDP_PORT "8889"

#define SPIN_RATE 100 // Change this later to ROS::Rate

using boost::asio::ip::udp;

class TelloDriver
{

    public:
        TelloDriver();
        ~TelloDriver();
    
        void activate_drone();
        void keep_drone_alive(); // Tello will autoland if nothing is heard for 15 seconds. We want to prevent that here.
        
        // Publishers
        ros::Publisher image_pub;
        ros::Publisher camera_info_pub;
        ros::Publisher  flight_data_pub;
        ros::Publisher tello_response_pub;

    private:
        ros::NodeHandle nh;
        void spin_1s();

       // void command_callback(ros_tello_driver::TelloAction::request, ros_tello_driver::TelloAction:: response);


       


 
};
