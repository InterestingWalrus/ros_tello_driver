#include <ros/ros.h>
#include <thread>
#include  <iostream>
#include <mutex>
#include <boost/asio.hpp>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include <ros_tello_driver/FlightTelemetry.h>
#include <ros_tello_driver/TelloAction.h>
#include <ros_tello_driver/TelloResponse.h>

// #include "ros_tello_driver/tello_socket.h"
class TelloCommandSocket;
class TelloStateSocket;
class TelloVideoSocket;


#define IP_ADDRESS "192.168.10.1"
#define CMD_UDP_PORT "8889"

#define SPIN_RATE 100 // Change this later to ROS::Rate

const int STATE_TIMEOUT = 4; // Stoppped received Telemetry data
const int VIDEO_TIMEOUT = 4; //Stopped receiving video;
const int KEEP_DRONE_ALIVE = 12; //keep sending input to Drone
const int COMMAND_TIMEOUT = 0; // Drone didn;t respond to command;


using boost::asio::ip::udp;

class TelloDriver
{

    public:
        TelloDriver();
        ~TelloDriver();
    
        void activate_drone();
       
        
        // Publishers
        ros::Publisher image_pub;
        ros::Publisher camera_info_pub;
        ros::Publisher  flight_data_pub;
        ros::Publisher tello_response_pub;

    private:
        ros::NodeHandle nh;
        void spin_1s();
        void keep_drone_alive(); // Tello will autoland if nothing is heard for 15 seconds. We want to prevent that here.

        //  std::unique_ptr<TelloCommandSocket> command_socket;
        //  std::unique_ptr<TelloVideoSocket> video_socket;
        //  std::unique_ptr<TelloStateSocket> state_socket;

        TelloCommandSocket* command_socket;
        TelloVideoSocket* video_socket;
        TelloStateSocket* state_socket;
    
        
        ros::Subscriber cmd_vel_sub;
        ros::ServiceClient command_service;
        void command_callback(ros_tello_driver::TelloAction::Request &request, ros_tello_driver::TelloAction::Response &response);
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
        
        


       


 
};
