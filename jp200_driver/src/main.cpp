#include <ros/ros.h>
#include <jp200_driver/JP200MultiArray.h>


void callback(const jp200_driver::JP200MultiArray::ConstPtr& msg)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jp200_driver");

    ros::NodeHandle nh;
}