#include <ros/ros.h>
#include <jp200_driver/JP200MultiArray.h>

#include <string>
#include <vector>
#include "jp200_driver/utils.hpp"

using namespace jp200_driver;

class ROS1JP200Driver
{
    public:
    ROS1JP200Driver(ros::NodeHandle nh):nh_(nh)
    {
        ROS_INFO_STREAM("Set Parameters");
        ros::param::set("port_name", "/dev/ttyUSB0");
        ros::param::set("baud_rate", 1000000);
        ros::param::set("enable_response", true);
        ros::param::set("servo_num", 1);
        ros::param::get("port_name", port_name_);
        ros::param::get("baud_rate", baud_rate_);
        ros::param::get("enable_response", enable_response_);
        ros::param::get("servo_num", servo_num_);

        utils_ = std::shared_ptr<jp200_driver::JP200Utils>(jp200_driver::JP200Utils::getJP200Utils(port_name_, baud_rate_));

        for(int i = 0; i < servo_num_; i++)
        {
            auto cmd = JP200Utils::JP200Cmd();
            commands.push_back(cmd);
        }
    }

    std::string port_name_;
    int baud_rate_, servo_num_;
    bool enable_response_;
    std::shared_ptr<jp200_driver::JP200Utils> utils_;
    std::vector<jp200_driver::JP200Utils::JP200Cmd> commands;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

};

void callback(const jp200_driver::JP200MultiArray::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jp200_driver");
    ros::NodeHandle nh;

    

    

    ROS_INFO_STREAM("Init subscriber");
    ros::Subscriber cmd_subscriber_ = nh.subscribe("jp200_cmd", 1000, callback);

    utils->open_port();

    if(utils->get_fd() < 0)
    {
        ROS_ERROR_STREAM("Failed to open port");
        utils->close_port();
    }
    else
    {
        ROS_INFO_STREAM("Serial port was connected");
    }
}