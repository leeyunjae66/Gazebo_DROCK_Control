#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

using namespace std::chrono_literals;

class CmdVelTrackControlNode : public rclcpp::Node
{
public:
  CmdVelTrackControlNode()
  : Node("cmd_vel_track_control_node")
  {
    const auto cmd_vel_topic =
      this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    const auto gazebo_topic =
      this->declare_parameter<std::string>(
        "gazebo_cmd_vel_topic",
        "/gazebo/drok_gazebo/cmd_vel_twist");

    watchdog_timeout_sec_ =
      this->declare_parameter<double>("watchdog_timeout_sec", 0.30);

    gazebo_node_.reset(new gazebo::transport::Node());

    // Gazebo master에 기본 namespace로 연결
    gazebo_node_->Init();

    // 절대 Gazebo transport 토픽으로 직접 발행
    gazebo_pub_ =
      gazebo_node_->Advertise<gazebo::msgs::Twist>(gazebo_topic);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      rclcpp::QoS(10),
      std::bind(
        &CmdVelTrackControlNode::cmdVelCallback,
        this,
        std::placeholders::_1));

    last_rx_time_ = this->now();

    watchdog_timer_ = this->create_wall_timer(
      50ms,
      std::bind(&CmdVelTrackControlNode::watchdogCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Bridge ready: ROS %s -> Gazebo %s",
      cmd_vel_topic.c_str(),
      gazebo_topic.c_str());
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_rx_time_ = this->now();
    received_command_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "RX /cmd_vel: linear_x=%.3f, angular_z=%.3f",
      msg->linear.x,
      msg->angular.z);

    publishToGazebo(msg->linear.x, -msg->angular.z);
  }

  void watchdogCallback()
  {
    if (!received_command_) {
      return;
    }

    const double elapsed = (this->now() - last_rx_time_).seconds();

    if (elapsed > watchdog_timeout_sec_) {
      RCLCPP_INFO(this->get_logger(), "Watchdog stop command");
      publishToGazebo(0.0, 0.0);
      received_command_ = false;
    }
  }

  void publishToGazebo(double linear_x, double angular_z)
  {
    gazebo::msgs::Twist gazebo_msg;

    gazebo_msg.mutable_linear()->set_x(linear_x);
    gazebo_msg.mutable_linear()->set_y(0.0);
    gazebo_msg.mutable_linear()->set_z(0.0);

    gazebo_msg.mutable_angular()->set_x(0.0);
    gazebo_msg.mutable_angular()->set_y(0.0);
    gazebo_msg.mutable_angular()->set_z(angular_z);

    gazebo_pub_->Publish(gazebo_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "TX Gazebo: linear_x=%.3f, angular_z=%.3f",
      linear_x,
      angular_z);
  }

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::PublisherPtr gazebo_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_rx_time_;
  double watchdog_timeout_sec_{0.30};
  bool received_command_{false};
};

int main(int argc, char * argv[])
{
  char gazebo_arg0[] = "cmd_vel_track_control_node";
  char * gazebo_argv[] = {gazebo_arg0, nullptr};

  gazebo::client::setup(1, gazebo_argv);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelTrackControlNode>());
  rclcpp::shutdown();

  gazebo::client::shutdown();
  return 0;
}
