#include "ros_tello_driver/tello_socket.h"
#include "ros_tello_driver/tello_driver.h"

void TelloSocket::listen()
{
    thread_ = std::thread(
    [this]()
    {
       for(;;)
       {
         size_t r = socket_.receive(boost::asio::buffer(buffer_));
         process_packet(r);       
       }
    });

}

bool TelloSocket::receiving()
{
    std::lock_guard<std::mutex>lock(mutex_);

    return rx;
}

ros::Time TelloSocket::recv_time()
{
    std::lock_guard<std::mutex>lock(mutex_);
    return recv_time_;
}

void TelloSocket::timeout()
{
    std::lock_guard<std::mutex>lock(mutex_);
    // Set receiving to false if we encounter a timeout
    rx = false;
}