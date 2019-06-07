#include "ros_tello_driver/tello_socket.h"
#include "ros_tello_driver/h264_decoder.h"
#include <sensor_msgs/CameraInfo.h>


class TelloVideoSocket: public TelloSocket
{
    private:

        void process_packet(size_t packet) override;  // overide lets compiler know that you are overriding the method is a virtual method of TelloSocket
        void decode_frames();

        std::vector<unsigned char> video_seq_buffer; // Compile video packets into larger sequence
        size_t next_seq_buffer = 0; // Next available position in video buffer
        int seq_buffer_packet_len = 0;    // how many packets collected

        H264Decoder decoder;  // H264 decoder class object
        ConvertRGB24 converter; // converts from YUV420P to BGR24

        sensor_msgs::CameraInfo camera_info_msg;



    public:
        TelloVideoSocket(TelloDriver *driver, unsigned short video_port);


};