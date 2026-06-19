#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

class FlipperJoyControlNode : public rclcpp::Node
{
public:
  FlipperJoyControlNode()
  : Node("flipper_joy_control_node")
  {
    // ================================
    // Parameters
    // ================================
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");

    // position controller command topic
    this->declare_parameter<std::string>(
      "command_topic",
      "/flipper_position_controller/commands"
    );

    // Xbox 계열 기본 기준
    // axes[7] : D-pad 위/아래 -> 앞 플리퍼 FRF, FLF
    // axes[6] : D-pad 좌/우   -> 뒤 플리퍼 BRF, BLF
    this->declare_parameter<int>("front_axis", 7);
    this->declare_parameter<int>("rear_axis", 6);

    // A 버튼: 현재 각도에서 즉시 hold
    this->declare_parameter<int>("hold_button", 0);

    // B 버튼: 플리퍼 목표 각도 0으로 복귀
    this->declare_parameter<int>("reset_button", 1);

    // 조이스틱을 누르고 있을 때 목표 각도가 변하는 속도 [rad/s]
    this->declare_parameter<double>("angle_rate", 1.0);

    // 플리퍼 목표 각도 제한 [rad]
    this->declare_parameter<double>("min_angle", -1.57);
    this->declare_parameter<double>("max_angle", 1.57);

    // 조이스틱 deadzone
    this->declare_parameter<double>("deadzone", 0.2);

    // ================================
    // 방향 보정
    // ================================
    // controller.yaml joint 순서:
    // FRF_joint, FLF_joint, BRF_joint, BLF_joint
    //
    // 네가 수동 테스트에서
    // data: [1.0, 1.0, 0.0, 0.0]
    // 이 FRF + FLF를 같은 방향으로 움직였다고 했으므로
    // 기본값은 전부 +1.0으로 둔다.
    //
    // 만약 나중에 한쪽이 반대로 움직이면 그쪽 sign만 -1.0으로 바꾸면 됨.
    this->declare_parameter<double>("frf_sign", 1.0);
    this->declare_parameter<double>("flf_sign", -1.0);
    this->declare_parameter<double>("brf_sign", 1.0);
    this->declare_parameter<double>("blf_sign", -1.0);

    joy_topic_ = this->get_parameter("joy_topic").as_string();
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    command_topic_ = this->get_parameter("command_topic").as_string();

    front_axis_ = this->get_parameter("front_axis").as_int();
    rear_axis_ = this->get_parameter("rear_axis").as_int();

    hold_button_ = this->get_parameter("hold_button").as_int();
    reset_button_ = this->get_parameter("reset_button").as_int();

    angle_rate_ = this->get_parameter("angle_rate").as_double();
    min_angle_ = this->get_parameter("min_angle").as_double();
    max_angle_ = this->get_parameter("max_angle").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();

    frf_sign_ = this->get_parameter("frf_sign").as_double();
    flf_sign_ = this->get_parameter("flf_sign").as_double();
    brf_sign_ = this->get_parameter("brf_sign").as_double();
    blf_sign_ = this->get_parameter("blf_sign").as_double();

    // 초기 목표 각도
    front_target_angle_ = 0.0;
    rear_target_angle_ = 0.0;

    last_time_ = this->now();

    // ================================
    // Subscriber / Publisher
    // ================================
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&FlipperJoyControlNode::joyCallback, this, std::placeholders::_1)
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      10,
      std::bind(&FlipperJoyControlNode::jointStateCallback, this, std::placeholders::_1)
    );

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_,
      10
    );

    // 50Hz로 위치 명령 계속 publish
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&FlipperJoyControlNode::update, this)
    );

    RCLCPP_INFO(this->get_logger(), "flipper_joy_control_node started");
    RCLCPP_INFO(this->get_logger(), "Subscribing joy         : %s", joy_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing joint_state : %s", joint_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing position cmd : %s", command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "front_axis: %d, rear_axis: %d", front_axis_, rear_axis_);
    RCLCPP_INFO(this->get_logger(), "angle_rate: %.3f rad/s", angle_rate_);
    RCLCPP_INFO(this->get_logger(), "angle limit: %.3f ~ %.3f rad", min_angle_, max_angle_);
    RCLCPP_INFO(
      this->get_logger(),
      "signs: FRF %.1f, FLF %.1f, BRF %.1f, BLF %.1f",
      frf_sign_, flf_sign_, brf_sign_, blf_sign_
    );
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_joy_ = msg;
    joy_received_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        continue;
      }

      const std::string & name = msg->name[i];
      const double pos = msg->position[i];

      if (name == "FRF_joint") {
        current_pos_[0] = pos;
        current_pos_received_[0] = true;
      } else if (name == "FLF_joint") {
        current_pos_[1] = pos;
        current_pos_received_[1] = true;
      } else if (name == "BRF_joint") {
        current_pos_[2] = pos;
        current_pos_received_[2] = true;
      } else if (name == "BLF_joint") {
        current_pos_[3] = pos;
        current_pos_received_[3] = true;
      }
    }

    const bool all_received =
      current_pos_received_[0] &&
      current_pos_received_[1] &&
      current_pos_received_[2] &&
      current_pos_received_[3];

    if (all_received) {
      joint_state_received_ = true;

      // 처음 joint state를 받으면 현재 각도를 목표 각도로 초기화
      // 갑자기 0도로 튀는 것을 방지
      if (!target_initialized_) {
        setTargetToCurrentPosition();
        target_initialized_ = true;

        RCLCPP_INFO(
          this->get_logger(),
          "Target initialized from current joint states. front: %.3f, rear: %.3f",
          front_target_angle_,
          rear_target_angle_
        );
      }
    }
  }

  double getAxis(int index) const
  {
    if (!latest_joy_) {
      return 0.0;
    }

    if (index < 0 || index >= static_cast<int>(latest_joy_->axes.size())) {
      return 0.0;
    }

    const double value = latest_joy_->axes[index];

    if (std::fabs(value) < deadzone_) {
      return 0.0;
    }

    return value;
  }

  bool getButton(int index) const
  {
    if (!latest_joy_) {
      return false;
    }

    if (index < 0 || index >= static_cast<int>(latest_joy_->buttons.size())) {
      return false;
    }

    return latest_joy_->buttons[index] == 1;
  }

  double clamp(double value, double min_value, double max_value) const
  {
    return std::max(min_value, std::min(value, max_value));
  }

  void setTargetToCurrentPosition()
  {
    if (!joint_state_received_) {
      return;
    }

    // sign을 고려해서 현재 joint 각도를 물리적인 front/rear 목표각도로 변환
    const double front_from_frf = frf_sign_ * current_pos_[0];
    const double front_from_flf = flf_sign_ * current_pos_[1];

    const double rear_from_brf = brf_sign_ * current_pos_[2];
    const double rear_from_blf = blf_sign_ * current_pos_[3];

    front_target_angle_ = 0.5 * (front_from_frf + front_from_flf);
    rear_target_angle_ = 0.5 * (rear_from_brf + rear_from_blf);

    front_target_angle_ = clamp(front_target_angle_, min_angle_, max_angle_);
    rear_target_angle_ = clamp(rear_target_angle_, min_angle_, max_angle_);
  }

  void update()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.2) {
      dt = 0.02;
    }

    if (joy_received_) {
      const bool hold_pressed = getButton(hold_button_);
      const bool reset_pressed = getButton(reset_button_);

      if (hold_pressed) {
        // A 버튼: 현재 플리퍼 각도를 목표각도로 저장해서 즉시 hold
        setTargetToCurrentPosition();
      } else if (reset_pressed) {
        // B 버튼: 목표 각도 0으로 복귀
        front_target_angle_ = 0.0;
        rear_target_angle_ = 0.0;
      } else {
        const double front_input = getAxis(front_axis_);
        const double rear_input = getAxis(rear_axis_);

        // position controller이므로 속도 명령을 직접 보내지 않는다.
        // 조이스틱 입력으로 목표 각도를 누적해서 바꾼다.
        front_target_angle_ += front_input * angle_rate_ * dt;
        rear_target_angle_ += rear_input * angle_rate_ * dt;

        front_target_angle_ = clamp(front_target_angle_, min_angle_, max_angle_);
        rear_target_angle_ = clamp(rear_target_angle_, min_angle_, max_angle_);
      }
    }

    std_msgs::msg::Float64MultiArray cmd;

    // controller.yaml의 joint 순서:
    // data[0] = FRF_joint
    // data[1] = FLF_joint
    // data[2] = BRF_joint
    // data[3] = BLF_joint
    //
    // 조이스틱을 놓아도 front_target_angle_, rear_target_angle_은 유지됨.
    // 따라서 position controller가 해당 각도를 계속 유지하려고 함.
    cmd.data = {
      frf_sign_ * front_target_angle_,
      flf_sign_ * front_target_angle_,
      brf_sign_ * rear_target_angle_,
      blf_sign_ * rear_target_angle_
    };

    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "target front: %.3f, rear: %.3f -> FRF %.3f, FLF %.3f, BRF %.3f, BLF %.3f",
      front_target_angle_,
      rear_target_angle_,
      cmd.data[0],
      cmd.data[1],
      cmd.data[2],
      cmd.data[3]
    );
  }

private:
  std::string joy_topic_;
  std::string joint_state_topic_;
  std::string command_topic_;

  int front_axis_;
  int rear_axis_;
  int hold_button_;
  int reset_button_;

  double angle_rate_;
  double min_angle_;
  double max_angle_;
  double deadzone_;

  double frf_sign_;
  double flf_sign_;
  double brf_sign_;
  double blf_sign_;

  double front_target_angle_;
  double rear_target_angle_;

  bool joy_received_ = false;
  bool joint_state_received_ = false;
  bool target_initialized_ = false;

  std::array<double, 4> current_pos_ = {0.0, 0.0, 0.0, 0.0};
  std::array<bool, 4> current_pos_received_ = {false, false, false, false};

  rclcpp::Time last_time_;

  sensor_msgs::msg::Joy::SharedPtr latest_joy_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlipperJoyControlNode>());
  rclcpp::shutdown();
  return 0;
}
