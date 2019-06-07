#include "ros_tello_driver/tello_socket.h"


class TelloStateSocket : public TelloSocket
{

    public:
        TelloStateSocket(TelloDriver *driver, unsigned short data_port);

    private:

       void process_packet(size_t packet) override; // Virtual method overriding the base class TelloSocket
      // uint8_t sdk_ver =  tello_msgs::msg::FlightData::SDK_UNKNOWN;  // Tello SDK version

};