#include "ros/ros.h"
#include "jp200_driver/JP200MultiArray.h"
#include "jp200_driver/JP200.h"

class ExampleNode {
public:
  ExampleNode() {
    pub_ = nh_.advertise<jp200_driver::JP200MultiArray>("jp200_cmd", 10);

    timer_ = nh_.createTimer(ros::Duration(1.0), &ExampleNode::timerCallback, this);
    count = 0;
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  size_t count;

  void timerCallback(const ros::TimerEvent& event) {
    jp200_driver::JP200MultiArray pub_msg;
    pub_msg.servo_num = 1;
    
    jp200_driver::JP200 msg;
    msg.id = 1;
    msg.control_mode = 1;

    msg.enable_pwm  =true;
    msg.pwm_cmd = 10;

    msg.angle_cmd.enable = true;
    msg.angle_cmd.value = count;

    pub_msg.servos.push_back(msg);

    pub_.publish(pub_msg);
    count += 10;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_node");

  ExampleNode example_node;

  example_node.run();

  return 0;
}
