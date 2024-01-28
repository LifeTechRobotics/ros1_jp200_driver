#include <ros/ros.h>
#include <jp200_driver/JP200MultiArray.h>

#include <string>
#include <vector>
#include "jp200_driver/utils.hpp"

using namespace jp200_driver;

void callback(const jp200_driver::JP200MultiArray::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jp200_driver");
    ros::NodeHandle nh;

    std::string port_name_;
    int baud_rate_, servo_num_;
    bool enable_response_;
    std::shared_ptr<jp200_driver::JP200Utils> utils;
    std::vector<jp200_driver::JP200Utils::JP200Cmd> commands;

    ros::param::set("port_name", "/dev/ttyUSB0");
    ros::param::set("baud_rate", 1000000);
    ros::param::set("enable_response", true);
    ros::param::set("servo_num", 1);
    ros::param::get("port_name", port_name_);
    ros::param::get("baud_rate", baud_rate_);
    ros::param::get("enable_response", enable_response_);
    ros::param::get("servo_num", servo_num_);

    utils = std::shared_ptr<jp200_driver::JP200Utils>(jp200_driver::JP200Utils::getJP200Utils(port_name_, baud_rate_));

    for(int i = 0; i < servo_num_; i++)
    {
        auto cmd = JP200Utils::JP200Cmd();
        commands.push_back(cmd);
    }
}