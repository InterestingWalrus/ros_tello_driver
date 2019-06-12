#include "ros_tello_driver/tello_socket.h"

class TelloCommandSocket : public TelloSocket
{
    public:
        TelloCommandSocket(TelloDriver *driver, std::string drone_ip, unsigned short drone_port, unsigned short command_port);
        
        void timeout() override;
        bool waiting();
        ros::Time tx_time();
        void send_command(std::string command, bool respond);


    private:
       
       void process_packet(size_t r) override;
       void complete_command(uint8_t rc, std::string str );

       udp::endpoint remote_endpoint;

       ros::Time latest_send_time; // time of latest TX
       bool response_; // Sent response on tello_response
       bool wait_ = false; // Waiting for a response?


};