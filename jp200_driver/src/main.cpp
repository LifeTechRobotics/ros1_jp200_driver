#include <ros/ros.h>
#include <jp200_driver/JP200MultiArray.h>
#include <jp200_driver/JP200Responses.h>

#include <string>
#include <vector>
#include "jp200_driver/utils.hpp"

using namespace jp200_driver;

class ROS1JP200Driver
{
    public:
    ROS1JP200Driver()
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
            commands_.push_back(cmd);
        }

        ROS_INFO_STREAM("Init ROS Node");
        pub_ = nh_.advertise<jp200_driver::JP200Responses>("jp200_responses", 10);
        sub_ = nh_.subscribe("jp200_cmd", 10, &ROS1JP200Driver::topic_callback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0), &ROS1JP200Driver::timer_callback, this);

        ROS_INFO_STREAM("Open port");
        utils_->open_port();

        if(utils_->get_fd() < 0)
        {
            ROS_ERROR_STREAM("Failed to open port");
            utils_->close_port();
        }
        else
        {
            ROS_INFO_STREAM("Serial port was connected");
        }
    }

    void run(){
        ros::spin();
    }

    private:
    void topic_callback(const jp200_driver::JP200MultiArray::ConstPtr& msg)
    {
        for(int i = 0; i < servo_num_; i++)
        {
            commands_[i].id = msg->servos[i].id;
            commands_[i].control_mode = msg->servos[i].control_mode;

            commands_[i].angle.enable = msg->servos[i].angle_cmd.enable;
            commands_[i].angle.value = msg->servos[i].angle_cmd.value;

            commands_[i].velocity.enable = msg->servos[i].velocity_cmd.enable;
            commands_[i].velocity.value = msg->servos[i].velocity_cmd.value;
            commands_[i].current.enable = msg->servos[i].current_cmd.enable;
            commands_[i].current.value = msg->servos[i].current_cmd.value;

            commands_[i].pwm_enable = msg->servos[i].enable_pwm;
            commands_[i].pwm_rate = msg->servos[i].pwm_cmd;

            commands_[i].position_gain.enable = msg->servos[i].position_gain.enable;
            commands_[i].position_gain.p = msg->servos[i].position_gain.p;
            commands_[i].position_gain.i = msg->servos[i].position_gain.i;
            commands_[i].position_gain.d = msg->servos[i].position_gain.d;
            commands_[i].position_gain.f = msg->servos[i].position_gain.f;

            commands_[i].velocity_gain.enable = msg->servos[i].velocity_gain.enable;
            commands_[i].velocity_gain.p = msg->servos[i].velocity_gain.p;
            commands_[i].velocity_gain.i = msg->servos[i].velocity_gain.i;
            commands_[i].velocity_gain.d = msg->servos[i].velocity_gain.d;
            commands_[i].velocity_gain.f = msg->servos[i].velocity_gain.f;

            commands_[i].current_gain.enable = msg->servos[i].current_gain.enable;
            commands_[i].current_gain.p = msg->servos[i].current_gain.p;
            commands_[i].current_gain.i = msg->servos[i].current_gain.i;
            commands_[i].current_gain.d = msg->servos[i].current_gain.d;
            commands_[i].current_gain.f = msg->servos[i].current_gain.f;
        }

        utils_->createJp200Cmd(commands_, enable_response_);
        int err = utils_->write_serial();
        if(err > 0)
        {
            ROS_INFO_STREAM(utils_->get_tx_packet().c_str());
        }
        else
        {
            ROS_INFO_STREAM("Failed to write");
        }
    }

    void timer_callback(const ros::TimerEvent& event)
    {
        if(enable_response_)
        {
            int error = utils_->read_serial();
            if(error > 0)
            {
                ROS_INFO_STREAM(utils_->get_rx_packet().c_str());
                
                resps_ = utils_->getResponse(servo_num_);
                jp200_driver::JP200Responses pub_msg;
                for(int i = 0; i < servo_num_; i++)
                {
                    jp200_driver::JP200Response msg;
                    msg.id = resps_[i].id;
                    msg.control_mode = resps_[i].control_mode;

                    msg.target_angle = resps_[i].target_angle;
                    msg.target_velocity = resps_[i].target_velocity;
                    msg.target_current = resps_[i].target_current;
                    msg.target_pwm = resps_[i].target_pwm;

                    msg.angle_feedback = resps_[i].angle_feedback;
                    msg.velocity_feedback = resps_[i].velocity_feedback;
                    msg.current_feedback = resps_[i].current_feedback;
                    msg.pwm_feedback = resps_[i].pwm_feedback;

                    msg.mpu_temp_feedback =resps_[i].mpu_temp_feedback;
                    msg.amp_temp_feedback =resps_[i].amp_temp_feedback;
                    msg.motor_temp_feedback =resps_[i].motor_temp_feedback;

                    msg.voltage_feedback = resps_[i].voltage_feedback;
                    msg.status_feedback = resps_[i].status_feedback;

                    msg.target_position_gain = resps_[i].target_position_gain;
                    msg.target_velocity_gain = resps_[i].target_velocity_gain;
                    msg.target_current_gain = resps_[i].target_current_gain;

                    pub_msg.responses.push_back(msg);
                    pub_msg.servo_num++;
                }

                pub_.publish(pub_msg);
            }
        }
    }

    std::string port_name_;
    int baud_rate_, servo_num_;
    bool enable_response_;
    std::shared_ptr<jp200_driver::JP200Utils> utils_;
    std::vector<jp200_driver::JP200Utils::JP200Cmd> commands_;
    std::vector<jp200_driver::JP200Utils::Response> resps_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Timer timer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jp200_driver");

    ROS1JP200Driver node;

    node.run();

    return 0;
}