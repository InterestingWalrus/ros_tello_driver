#include "ros_tello_driver/tello_command_socket.h"
#include "ros_tello_driver/tello_driver.h"

TelloCommandSocket::TelloCommandSocket(TelloDriver *driver, std::string drone_ip, unsigned short drone_port, unsigned short command_port) : TelloSocket(driver, command_port),
                    remote_endpoint(boost::asio::ip::address_v4::from_string(drone_ip),drone_port), latest_send_time(ros::Time::now())
{
   buffer_ = std::vector<unsigned char>(1024);
   listen();
}

void TelloCommandSocket::timeout()
{
    std::lock_guard<std::mutex>lock(mutex_);
    rx = false;

    if(wait_)
    {
        complete_command(ros_tello_driver::TelloResponse::TIMEOUT, "error: command timed out");
    }
}

bool TelloCommandSocket::waiting()
{
    std::lock_guard<std::mutex>lock(mutex_);
    return wait_;
}

ros::Time TelloCommandSocket::tx_time()
{
    std::lock_guard<std::mutex>lock(mutex_);
    return latest_send_time;
}

void TelloCommandSocket::send_command(std::string command, bool respond)
{
    std::lock_guard<std::mutex>lock(mutex_);

    if(!wait_)
    {
        ROS_DEBUG("Sending %s ----", command.c_str());
        socket_.send_to(boost::asio::buffer(command), remote_endpoint);
        latest_send_time = ros::Time::now();
    }

    // Wait for response from drones
    //rc rc won;t send command so don't wait for it
    if(command.rfind("rc", 0) != 0)
    {
        response_ = respond;
        wait_ = true;
    }
}

void TelloCommandSocket::complete_command(uint8_t rc, std::string str)
{
    if(response_)
    {
        ros_tello_driver::TelloResponse response_msg;
        response_msg.rc = rc;
        response_msg.str = str;
        tello_driver->tello_response_pub.publish(response_msg);
    }

    wait_ = false;

}

void TelloCommandSocket::process_packet(size_t r)
{
    std::lock_guard<std::mutex>lock(mutex_);

    recv_time_ = ros::Time::now();

    if(!rx)
    {
        rx = true;
    }

    std::string str = std::string(buffer_.begin(), buffer_.begin() + r);
    if(wait_)
    {
        ROS_DEBUG("received '%s'", str.c_str());
        complete_command(str=="error"? ros_tello_driver::TelloResponse::ERROR : ros_tello_driver::TelloResponse::OK, str);
    }
    else
    {
        ROS_WARN("Unexptected '%s'", str.c_str());
    }
}