#include <fstream>
#include "sensor_msgs/CameraInfo.h"
#include <ros/ros.h>

std::string camera_cfg_path("../cfg/camera_info.txt");

// Camera Calibration parameters

bool get_camera_info(sensor_msgs::CameraInfo &info)
{
  // File format: 2 ints and 9 floats, separated by whitespace:
  // height width fx fy cx cy k1 k2 t1 t2 t3

    std::ifstream file;

    file.open(camera_cfg_path);
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