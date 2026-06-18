#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <string>

class CmdVelTrackControlNode : public rclcpp::Node
{
public:
  CmdVelTrackControlNode()
  : Node("cmd_vel_track_control_node")
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("command_topic", "/track_velocity_controller/commands");

    // URDF 기준:
    // FL/FR y 위치: +0.1836 / -0.1836
    // 좌우 바퀴 간 거리 = 0.1836 * 2 = 0.3672 m
    this->declare_parameter<double>("wheel_separation", 0.3672);

    // URDF collision cylinder radius
    this->declare_parameter<double>("wheel_radius", 0.095);

    // 최대 바퀴 각속도 제한 [rad/s]
    // 직접 테스트에서 25 rad/s가 잘 움직였으므로 여유 있게 50으로 설정
    this->declare_parameter<double>("max_wheel_rad_s", 4.0);

    // 조이스틱 / cmd_vel 입력 배율
    // teleop_twist_joy에서 angular.z = -1.5 정도가 들어오면
    // 기존 계산으로는 약 2.9 rad/s밖에 안 나와서 너무 약했음.
    // angular_gain = 8.0이면 약 23 rad/s 정도로 증가.
    this->declare_parameter<double>("linear_gain", 1.0);
    this->declare_parameter<double>("angular_gain", 1.0);

    // 작은 조이스틱 노이즈 제거
    this->declare_parameter<double>("deadband", 0.03);

    // 방향 보정용 파라미터
    //
    // 현재 Gazebo 직접 테스트 결과:
    // 전진:   [-25,  25, -25,  25]
    // 우회전: [ 25,  25,  25,  25]
    // 좌회전: [-25, -25, -25, -25]
    //
    // 따라서 left_sign = 1.0, right_sign = -1.0 이 맞음.
    this->declare_parameter<double>("left_sign", 1.0);
    this->declare_parameter<double>("right_sign", -1.0);

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    command_topic_ = this->get_parameter("command_topic").as_string();

    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    max_wheel_rad_s_ = this->get_parameter("max_wheel_rad_s").as_double();

    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    deadband_ = this->get_parameter("deadband").as_double();

    left_sign_ = this->get_parameter("left_sign").as_double();
    right_sign_ = this->get_parameter("right_sign").as_double();

    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_,
      10
    );

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      std::bind(&CmdVelTrackControlNode::cmdVelCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "cmd_vel_track_control_node started");
    RCLCPP_INFO(this->get_logger(), "Subscribing: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "wheel_separation: %.4f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "wheel_radius: %.4f m", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "max_wheel_rad_s: %.3f rad/s", max_wheel_rad_s_);
    RCLCPP_INFO(this->get_logger(), "linear_gain: %.3f", linear_gain_);
    RCLCPP_INFO(this->get_logger(), "angular_gain: %.3f", angular_gain_);
    RCLCPP_INFO(this->get_logger(), "deadband: %.3f", deadband_);
    RCLCPP_INFO(this->get_logger(), "left_sign: %.1f, right_sign: %.1f", left_sign_, right_sign_);
  }

private:
  double applyDeadband(double value) const
  {
    if (std::abs(value) < deadband_) {
      return 0.0;
    }
    return value;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    double linear_x = msg->linear.x;      // m/s
    double angular_z = msg->angular.z;    // rad/s

    if (!std::isfinite(linear_x) || !std::isfinite(angular_z)) {
      RCLCPP_WARN(this->get_logger(), "Invalid cmd_vel: linear.x or angular.z is not finite");
      publishStop();
      return;
    }

    // 조이스틱 미세 노이즈 제거
    linear_x = applyDeadband(linear_x);
    angular_z = applyDeadband(angular_z);

    // 조이스틱 입력 배율 적용
    linear_x *= linear_gain_;
    angular_z *= angular_gain_;

    // Differential drive / skid-steer model
    //
    // v_left  = v - (L / 2) * w
    // v_right = v + (L / 2) * w
    //
    // v: robot linear velocity [m/s]
    // w: robot yaw angular velocity [rad/s]
    // L: wheel separation [m]
    const double left_linear_m_s =
      linear_x - (wheel_separation_ / 2.0) * angular_z;

    const double right_linear_m_s =
      linear_x + (wheel_separation_ / 2.0) * angular_z;

    // wheel angular velocity [rad/s]
    double left_rad_s = left_linear_m_s / wheel_radius_;
    double right_rad_s = right_linear_m_s / wheel_radius_;

    // URDF joint axis / 모델 방향 보정
    left_rad_s *= left_sign_;
    right_rad_s *= right_sign_;

    // safety clamp
    left_rad_s = std::clamp(left_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);
    right_rad_s = std::clamp(right_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);

    std_msgs::msg::Float64MultiArray cmd;

    // controllers.yaml의 joints 순서와 반드시 같아야 함.
    //
    // controllers.yaml:
    //   - FR_JOINT
    //   - FL_JOINT
    //   - BR_JOINT
    //   - BL_JOINT
    //
    // 따라서 data 순서:
    //   [FR, FL, BR, BL]
    cmd.data = {
      right_rad_s,  // FR_JOINT
      left_rad_s,   // FL_JOINT
      right_rad_s,  // BR_JOINT
      left_rad_s    // BL_JOINT
    };

    wheel_cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "cmd_vel scaled v=%.3f m/s w=%.3f rad/s | wheel rad/s L=%.3f R=%.3f | cmd=[%.3f, %.3f, %.3f, %.3f]",
      linear_x,
      angular_z,
      left_rad_s,
      right_rad_s,
      cmd.data[0],
      cmd.data[1],
      cmd.data[2],
      cmd.data[3]
    );
  }

  void publishStop()
  {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {0.0, 0.0, 0.0, 0.0};
    wheel_cmd_pub_->publish(cmd);
  }

private:
  std::string cmd_vel_topic_;
  std::string command_topic_;

  double wheel_separation_;
  double wheel_radius_;
  double max_wheel_rad_s_;

  double linear_gain_;
  double angular_gain_;
  double deadband_;

  double left_sign_;
  double right_sign_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelTrackControlNode>());
  rclcpp::shutdown();
  return 0;
}
