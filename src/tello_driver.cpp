#include "ros_tello_driver/tello_driver.h"

#include "ros_tello_driver/tello_state_socket.h"
#include "ros_tello_driver/tello_video_socket.h"
#include "ros_tello_driver/tello_command_socket.h"

#include<memory>


// // Create pointer to the sockets


TelloDriver::TelloDriver() 
{
     // Initialise publishers

     image_pub = nh.advertise<sensor_msgs::Image>("image_raw", 1);
     camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
     flight_data_pub = nh.advertise<ros_tello_driver::FlightTelemetry>("flight_data", 1);
     tello_response_pub = nh.advertise<ros_tello_driver::TelloResponse>("drone_response", 1);

     command_service = nh.serviceClient<ros_tello_driver::TelloAction>("tello_action");

     cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &TelloDriver::cmd_vel_callback, this);

     // Parameters

     std::string drone_ip_address;
     int drone_port;
     int command_port;
     int data_port;
     int video_port;

    //TODO GEt and set params here
     nh.setParam("drone_ip", std::string("192.168.0.1"));
     nh.setParam("drone_port", 8889);
     nh.setParam("data_port", 8890);
     nh.setParam("command_port", 38065);
     nh.setParam("video_port", 11111);

     nh.getParam("drone_ip",drone_ip_address);
     nh.getParam("drone_port", drone_port);
     nh.getParam("data_port", data_port);
     nh.getParam("command_port", command_port);
     nh.getParam("video_port", video_port);


    ROS_INFO("Drone at %s:%d", drone_ip_address.c_str(), drone_port);
    ROS_INFO("Listening for commands on localhost:%d", command_port);
    ROS_INFO("Listening for data on localhost:%d", data_port);
    ROS_INFO("Grabbing video from localhost:%d", video_port);

   // Create unique pointer to socket objects. No need to call new or delete. 
   // Smart allocation. sexy.. C++ 14 feature not supported in  ROS1

//    command_socket = std::make_unique<TelloCommandSocket>(this, drone_ip_address, drone_port, command_port );
//    state_socket = std::make_unique<TelloStateSocket>(this, data_port);
//    video_socket = std::make_unique<TelloVideoSocket>(this, video_port);

    // Stores on heap // remember to delete this when you are done with it..

     command_socket = new TelloCommandSocket(this, drone_ip_address, drone_port, command_port );
     state_socket = new TelloStateSocket(this, data_port);
     video_socket = new  TelloVideoSocket(this, video_port);

}

TelloDriver::~TelloDriver()
{
    delete command_socket, state_socket, video_socket ;
}


void TelloDriver::command_callback(ros_tello_driver::TelloAction::Request &request, ros_tello_driver::TelloAction::Response &response)
{

    if(!state_socket->receiving() || video_socket->receiving())
    {
        ROS_WARN("Not connected, dropping '%s's", request.cmd.c_str());
        response.rc = response.ERROR_NOT_CONNECTED;
    }

    else if(command_socket->waiting())
    {
        ROS_WARN("Drone is Busy, dropping '%s'", request.cmd.c_str() );
        response.rc = response.ERROR_BUSY;
    }

    else
    {
        command_socket->send_command(request.cmd, true);
        response.rc = response.OK;
        
    }

}
void TelloDriver::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    

    //TODO Specify velocity instead of JOYSTICK position.

    if(!command_socket->waiting())
    {
        std::ostringstream rc;

        rc << "rc " << static_cast<int>(round(msg->linear.y * -100)) << " " 
           << static_cast<int>(round((msg->linear.x * 100))) << " " 
           << static_cast<int>(round((msg->linear.z * 100))) << " " 
           <<static_cast<int>(round((msg->angular.z * -100))) ;

        command_socket->send_command(rc.str(), false);
    }

}

void TelloDriver::activate_drone()
{
    static unsigned int counter = 0;
    counter ++;

    if(counter % SPIN_RATE == 0)
    {
        spin_1s();
    }
}

void TelloDriver::spin_1s()
{
    /********************  STARTUP **************************
     // Start Drone
     // check drone states
     **********************/

     if(!state_socket->receiving() && !command_socket-> waiting())
     {
         // Initiate SDK with "command"
         command_socket->send_command("command", false);
         return;
     }

     
     if(state_socket->receiving() && !video_socket->receiving() && !command_socket->waiting())
     {
         // start video recording
         command_socket->send_command("streamon", false);

     }

     bool timeout = false;

     if(command_socket->waiting() && ros::Time::now() - command_socket->tx_time() > ros::Duration(COMMAND_TIMEOUT, 0) )
     {
         ROS_ERROR("Command timed out");
         command_socket->timeout();
         timeout = true;
     }

     if(state_socket->receiving() && ros::Time::now() - state_socket->recv_time() > ros::Duration(STATE_TIMEOUT, 0) )
     {
         ROS_ERROR("No State received for 5 seconds");
         state_socket->timeout();
         timeout = true;
     }

     if(video_socket->receiving() && ros::Time::now() - video_socket->recv_time() > ros::Duration(VIDEO_TIMEOUT, 0) )
     {
         ROS_ERROR("No video received for 5 seconds");
         video_socket->timeout();
         timeout = true;
     }

     if(timeout)
     {
         return;
     }

     keep_drone_alive();
}

void TelloDriver::keep_drone_alive()
{
    // Default state of Tello is to land if it receives nothing after 15 seconds.
    // So we keep sending data to keep the drone alive
    if(state_socket->receiving() && video_socket->receiving() && !command_socket->waiting() 
    && (ros::Time::now() - command_socket->tx_time() > ros::Duration(KEEP_DRONE_ALIVE, 0 )))
    {
        command_socket->send_command("rc 0 0 0 0", false);
        return;

    }
}




int main (int argc, char ** argv)
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   ros::init(argc, argv, "tello_ros_client ");

    TelloDriver telloDriver;

    ros::Rate loop_rate(SPIN_RATE);

    while(ros::ok())
   {

       telloDriver.activate_drone();
       ros::spinOnce();

       loop_rate.sleep();
   }

 

    return 0;
}

