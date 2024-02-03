#include "ros/ros.h"
#include "std_msgs/String.h"

class ExampleNode {
public:
  ExampleNode() {
    pub_ = nh_.advertise<std_msgs::String>("example_topic", 10);

    // タイマーコールバックの設定
    timer_ = nh_.createTimer(ros::Duration(1.0), &ExampleNode::timerCallback, this);
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Timer timer_;

  void timerCallback(const ros::TimerEvent& event) {
    // タイマーコールバックでメッセージを作成してパブリッシュ
    std_msgs::String msg;
    msg.data = "Hello, ROS!";
    pub_.publish(msg);
  }
};

int main(int argc, char **argv) {
  // ROSノードの初期化
  ros::init(argc, argv, "example_node");

  // ExampleNodeクラスのインスタンスを作成
  ExampleNode example_node;

  // ノードの実行
  example_node.run();

  return 0;
}
