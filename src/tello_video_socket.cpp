#include "ros_tello_driver/tello_video_socket.h"
#include "ros_tello_driver/tello_driver.h"
#include <cv_bridge/cv_bridge.h>
#include <libavutil/frame.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Notes on Tello video:
  // -- frames are always 960x720.
  // -- frames are split into UDP packets of length 1460.
  // -- normal frames are ~10k, or about 8 UDP packets.
  // -- keyframes are ~35k, or about 25 UDP packets.
  // -- keyframes are always preceded by an 8-byte UDP packet and a 13-byte UDP packet -- markers?
  // -- the h264 parser will consume the 8-byte packet, the 13-byte packet and the entire keyframe without
  //    generating a frame. Presumably the keyframe is stored in the parser and referenced later.

  TelloVideoSocket::TelloVideoSocket(TelloDriver *driver, unsigned short video_port) : TelloSocket(driver, video_port)
  {
      if(!get_camera_info(camera_info_msg))
      {
          ROS_ERROR("Cannot get camera info! Frames will not be calibrated");

      }

      buffer_ = std::vector<unsigned char>(2048);
      video_seq_buffer = std::vector<unsigned char>(65536);
      listen();
  }


void TelloVideoSocket::process_packet(size_t r)
{
    std::lock_guard<std::mutex>lock(mutex_);
    
    recv_time_ = ros::Time::now();

    if(!rx)
    {
        // First packet
        ROS_INFO("receiving video");
        rx = true;
        next_seq_buffer = 0;
        seq_buffer_packet_len = 0;
    }

    // check if we've filled frames to be processed
    if(next_seq_buffer + r >= video_seq_buffer.size())
    {
        ROS_ERROR("Video buffer overflowing, dropping sequence");
        next_seq_buffer = 0;
        seq_buffer_packet_len = 0;
        return;
    }

    std::copy(buffer_.begin(), buffer_.begin() + r, video_seq_buffer.begin() + next_seq_buffer);
    next_seq_buffer +=r;
    seq_buffer_packet_len++;

    // Check Packet size.
    // If less than 1460, we've reached last packet in sequence.
    if(r < 1460)
    {
        decode_frames();

        // reset packet numbers
        next_seq_buffer = 0;
        seq_buffer_packet_len = 0;
    }
}

// Decode frames 
void TelloVideoSocket::decode_frames()
{
    size_t next = 0;

    try
    {
        while (next < next_seq_buffer)
        {
           // Parse H264 Video

           ssize_t consumed = decoder.parse(video_seq_buffer.data() + next, next_seq_buffer - next);

           // Check if fframe is avalable
           if(decoder.is_frame_available())
           {
                // Decode frame
                const AVFrame &frame = decoder.decode_frame();

                 
                // COnvert Pixels from YUV420P to BGR24
               // int size = converter.predict_size(frame.width, frame.height);
               int size = decoder.return_size();
                unsigned char bgr24[size];
               // converter.convert(frame, bgr24);

                // Convert to OpenCV Matrix
                cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24};

                //Display using OpenCV
                cv::imshow("TELLO Frame", mat);
                cv::waitKey(1);

                // sync ros messages:
                auto time_stamp = ros::Time::now();

                if(tello_driver->image_pub.getNumSubscribers() > 0)
                {
                    std_msgs::Header header{};
                    header.frame_id = "camera_frame";
                    header.stamp =time_stamp;
                    cv_bridge::CvImage image{header, sensor_msgs::image_encodings::BGR8, mat};
                    tello_driver->image_pub.publish(image.toImageMsg());
                    
                }

                if(tello_driver->camera_info_pub.getNumSubscribers() > 0)
                {
                    camera_info_msg.header.stamp = time_stamp;
                    tello_driver->camera_info_pub.publish(camera_info_msg);
                }
            next +=consumed;

            
           }
        }
        
    }
    catch( std::runtime_error e)
    {
        ROS_ERROR("Decoding error: %s", e.what());
    }
    
}

bool TelloVideoSocket::get_camera_info(sensor_msgs::CameraInfo &info)
{
  // File format: 2 ints and 9 floats, separated by whitespace:
  // height width fx fy cx cy k1 k2 t1 t2 t3
  // TODO Change file path on the parameter server.

    std::ifstream file;
     

    file.open("/home/daniel/catkin_ws/src/ros_tello_driver/cfg/camera_info.txt");
    if(!file)
    {
        ROS_ERROR("Unable to open config file");
        return false;
    }

    uint32_t height, width;
    double fx, fy, cx, cy, k1, k2, t1, t2, t3;
    file >> height >> width;
    file >> fx >> fy;
    file >> cx >> cy;
    file >> k1 >> k2 >> t1 >> t2 >> t3;

    info.header.frame_id = "camera_frame";
    info.height = height;
    info.width = width;
    info.distortion_model = "plumb_bob";

    info.D.push_back(k1);
    info.D.push_back(k2);
    info.D.push_back(t1);
    info.D.push_back(t2);
    info.D.push_back(t3);

    info.K[0] = fx;
    info.K[1] = 0;
    info.K[2] = cx;
    info.K[3] = 0;
    info.K[4] = fy;
    info.K[5] = cy;
    info.K[6] = 0;
    info.K[7] = 0;
    info.K[8] = 1;

    info.P[0] = fx;
    info.P[1] = 0;
    info.P[2] = cx;
    info.P[3] = 0;
    info.P[4] = 0;
    info.P[5] = fy;
    info.P[6] = cy;
    info.P[7] = 0;
    info.P[8] = 0;
    info.P[9] = 0;
    info.P[10] = 1;
    info.P[11] = 0;

    return true;
}