#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JakaStateAdapter
{
public:
  JakaStateAdapter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_joint_topic", input_topic_, std::string("/jaka_driver/joint_position"));
    pnh.param<std::string>("output_joint_topic", output_topic_, std::string("/joint_states"));

    pub_ = nh.advertise<sensor_msgs::JointState>(output_topic_, 10);
    sub_ = nh.subscribe(input_topic_, 10, &JakaStateAdapter::jointCallback, this);

    ROS_INFO_STREAM("[JakaStateAdapter] input=" << input_topic_ << ", output=" << output_topic_);
  }

private:
  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (msg->position.size() < 6)
    {
      ROS_WARN_THROTTLE(5.0, "[JakaStateAdapter] joint_position has <6 elements");
      return;
    }

    sensor_msgs::JointState out;
    out.header.stamp = msg->header.stamp;
    out.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    out.position.resize(6);

    for (size_t i = 0; i < 6; ++i)
    {
      out.position[i] = msg->position[i];
    }

    pub_.publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaka_state_adapter_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  JakaStateAdapter adapter(nh, pnh);
  ros::spin();
  return 0;
}
