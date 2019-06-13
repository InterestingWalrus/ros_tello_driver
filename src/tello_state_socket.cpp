#include "ros_tello_driver/tello_state_socket.h"
#include "ros_tello_driver/tello_driver.h"


#include <regex>
#include<map>

  // * make data useful by parsing all documented fields
  // * future SDK versions might introduce new field types, so don't parse undocumented fields
  // * send the raw string as well


TelloStateSocket::TelloStateSocket(TelloDriver *driver, unsigned short data_port): TelloSocket(driver, data_port)
{
    buffer_ = std::vector<unsigned char>(1024);
    listen();
}

// Process State packets from drone, @10Hz
void TelloStateSocket::process_packet(size_t r)
{
    // lock mutex
    std::lock_guard<std::mutex> lock(mutex_);

    // Use an associative container between a key value and a mapped value
    // Map keeps it value based on a given key
    static std::map<uint8_t, std::string> sdk_map{
        {ros_tello_driver::FlightTelemetry::SDK_UNKNOWN, "unknown"},
         {ros_tello_driver::FlightTelemetry::SDK_1_3, "v1.3"},
          {ros_tello_driver::FlightTelemetry::SDK_2_0, "unknown"}};

    recv_time_ = ros::Time::now();

    if(rx && tello_driver->flight_data_pub.getNumSubscribers() == 0)
    {
        // Then we do nothing here
        return;
    }

     
    /************************* Example Tello String SDKv1.3: 
     “pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy%d;vgz:%d;templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%.2f; time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n”
    *****************************************************************/
     // First split string on ; and then : to generate a new map;

     std::map<std::string, std::string> sdk_string_fields;
     std::string raw_data(buffer_.begin(), buffer_.end() + r);

     // Regex Expression  ([^:]+):([^;]+);
      std::regex reg_exp("([^:]+):([^;]+);");
      for(auto i = std::sregex_iterator(raw_data.begin(), raw_data.end(), reg_exp);
         i!= std::sregex_iterator(); ++i)
         {
             auto match = *i;
             sdk_string_fields[match[1]] = match[2];
         }

    // Have we received first message?
    if(!rx)
    {
        rx = true;

        // Determine Tello SDK version.
        // SDK 2.0 sends mission ID Pad version as "mid" so just search 
        //for that in you string

        sdk_ver = ros_tello_driver::FlightTelemetry::SDK_1_3;
        auto i = sdk_string_fields.find("mid");
        if(i != sdk_string_fields.end() && i->second != "257")
        {
            sdk_ver = ros_tello_driver::FlightTelemetry::SDK_2_0;
        } 

        ROS_INFO("Receiing state, SDK Version:  %s", sdk_map[sdk_ver].c_str());
    }

    // If anyone subscribes to message, send ros messages:
    if(tello_driver->flight_data_pub.getNumSubscribers() > 0)
    {
        ros_tello_driver::FlightTelemetry msg;
        msg.header.stamp = recv_time_;
        msg.raw = raw_data;
        msg.sdk = sdk_ver;

        try
        {
            {
                if(sdk_ver == ros_tello_driver::FlightTelemetry::SDK_2_0)
                {
                    msg.mid = std::stoi(sdk_string_fields["mid"]);
                    msg.x = std::stoi(sdk_string_fields["x"]);
                    msg.y = std::stoi(sdk_string_fields["y"]);
                    msg.z = std::stoi(sdk_string_fields["z"]);
                }

                // TODO Add states from AnqiXu's repo here
                else
                {
                    msg.pitch = std::stoi(sdk_string_fields["pitch"]);
                    msg.roll = std::stoi(sdk_string_fields["roll"]);
                    msg.yaw = std::stoi(sdk_string_fields["yaw"]);
                    msg.vgx = std::stoi(sdk_string_fields["vgx"]);
                    msg.vgy = std::stoi(sdk_string_fields["vgy"]);
                    msg.vgz = std::stoi(sdk_string_fields["vgz"]);
                    msg.ltemp = std::stoi(sdk_string_fields["templ"]);
                    msg.htemp = std::stoi(sdk_string_fields["temph"]);
                    msg.tof = std::stoi(sdk_string_fields["tof"]);
                    msg.h = std::stoi(sdk_string_fields["h"]);
                    msg.battery = std::stoi(sdk_string_fields["bat"]);
                    msg.alti = std::stof(sdk_string_fields["baro"]);
                    msg.time = std::stoi(sdk_string_fields["time"]);
                    msg.agx = std::stof(sdk_string_fields["agx"]);
                    msg.agy = std::stof(sdk_string_fields["agy"]);
                    msg.agz = std::stof(sdk_string_fields["agz"]);

                }
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("Can't parse drone data");
        }

        tello_driver->flight_data_pub.publish(msg);
        
    }

}