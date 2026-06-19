#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
    // Topics
    // ================================
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>(
      "command_topic",
      "/flipper_position_controller/joint_trajectory"
    );

    // ================================
    // Joystick mapping
    // ================================
    // axes[7] : D-pad 위/아래 -> 앞 플리퍼 FRF, FLF
    // axes[6] : D-pad 좌/우   -> 뒤 플리퍼 BRF, BLF
    this->declare_parameter<int>("front_axis", 7);
    this->declare_parameter<int>("rear_axis", 6);

    // A: 현재 상태 hold
    this->declare_parameter<int>("hold_button", 0);

    // B: 0 rad 방향으로 천천히 복귀
    this->declare_parameter<int>("reset_button", 1);

    // ================================
    // Motion parameters
    // ================================
    // 1.0 rad/s × 0.2 s = 약 0.2 rad 명령 변화
    // 수동 테스트의 0.2 rad와 유사한 수준
    this->declare_parameter<double>("angle_rate", 1.0);

    this->declare_parameter<double>("min_angle", -1.45);
    this->declare_parameter<double>("max_angle", 1.45);
    this->declare_parameter<double>("deadzone", 0.2);

    // 하나의 trajectory가 목표 위치까지 가는 시간
    this->declare_parameter<double>("trajectory_duration", 0.25);

    // trajectory 발행 주기: 0.20 s = 5 Hz
    // 기존 50 Hz 전송은 trajectory를 너무 자주 교체했다.
    this->declare_parameter<double>("command_period", 0.20);

    // ================================
    // Joint direction correction
    // ================================
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

    trajectory_duration_ =
      this->get_parameter("trajectory_duration").as_double();

    command_period_ =
      this->get_parameter("command_period").as_double();

    frf_sign_ = this->get_parameter("frf_sign").as_double();
    flf_sign_ = this->get_parameter("flf_sign").as_double();
    brf_sign_ = this->get_parameter("brf_sign").as_double();
    blf_sign_ = this->get_parameter("blf_sign").as_double();

    last_control_time_ = std::chrono::steady_clock::now();
    last_command_time_ = last_control_time_;

    // /joy, /joint_states는 best effort publisher일 수 있으므로
    // SensorDataQoS(best effort)로 구독한다.
    const auto sensor_qos = rclcpp::SensorDataQoS();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      sensor_qos,
      std::bind(
        &FlipperJoyControlNode::joyCallback,
        this,
        std::placeholders::_1
      )
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      sensor_qos,
      std::bind(
        &FlipperJoyControlNode::jointStateCallback,
        this,
        std::placeholders::_1
      )
    );

    // Controller subscriber가 BEST_EFFORT여도 RELIABLE publisher와 연결 가능
    cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      command_topic_,
      rclcpp::QoS(10).reliable()
    );

    // 입력 적분은 50 Hz 유지
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&FlipperJoyControlNode::update, this)
    );

    RCLCPP_INFO(this->get_logger(), "flipper_joy_control_node started");
    RCLCPP_INFO(this->get_logger(), "Joy topic: %s", joy_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Joint state topic: %s",
      joint_state_topic_.c_str()
    );
    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory topic: %s",
      command_topic_.c_str()
    );
    RCLCPP_INFO(
      this->get_logger(),
      "angle_rate: %.2f rad/s | command period: %.2f s",
      angle_rate_,
      command_period_
    );
  }

private:
  static constexpr std::size_t JOINT_COUNT = 4;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_joy_ = msg;
    joy_received_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        continue;
      }

      const std::string & name = msg->name[i];
      const double position = msg->position[i];

      if (name == "FRF_joint") {
        current_pos_[0] = position;
        current_pos_received_[0] = true;
      } else if (name == "FLF_joint") {
        current_pos_[1] = position;
        current_pos_received_[1] = true;
      } else if (name == "BRF_joint") {
        current_pos_[2] = position;
        current_pos_received_[2] = true;
      } else if (name == "BLF_joint") {
        current_pos_[3] = position;
        current_pos_received_[3] = true;
      }
    }

    if (!target_initialized_ && allJointStatesReceived()) {
      setTargetsToCurrentPosition();
      target_initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Initial targets loaded from /joint_states."
      );
    }
  }

  bool allJointStatesReceived() const
  {
    return current_pos_received_[0] &&
           current_pos_received_[1] &&
           current_pos_received_[2] &&
           current_pos_received_[3];
  }

  void setTargetsToCurrentPosition()
  {
    if (!allJointStatesReceived()) {
      return;
    }

    target_pos_ = current_pos_;
  }

  double getAxis(int index) const
  {
    if (!latest_joy_) {
      return 0.0;
    }

    if (index < 0 ||
        index >= static_cast<int>(latest_joy_->axes.size()))
    {
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

    if (index < 0 ||
        index >= static_cast<int>(latest_joy_->buttons.size()))
    {
      return false;
    }

    return latest_joy_->buttons[index] == 1;
  }

  double clamp(double value, double min_value, double max_value) const
  {
    return std::clamp(value, min_value, max_value);
  }

  double moveToward(
    double current,
    double target,
    double max_step
  ) const
  {
    if (current < target) {
      return std::min(current + max_step, target);
    }

    return std::max(current - max_step, target);
  }

  double getJointSign(std::size_t index) const
  {
    switch (index) {
      case 0:
        return frf_sign_;
      case 1:
        return flf_sign_;
      case 2:
        return brf_sign_;
      case 3:
        return blf_sign_;
      default:
        return 1.0;
    }
  }

  double getPhysicalTarget(std::size_t index) const
  {
    return getJointSign(index) * target_pos_[index];
  }

  void setPhysicalTarget(std::size_t index, double physical_target)
  {
    const double limited_target = clamp(
      physical_target,
      min_angle_,
      max_angle_
    );

    target_pos_[index] =
      getJointSign(index) * limited_target;
  }

  void addPhysicalTarget(std::size_t index, double delta)
  {
    setPhysicalTarget(
      index,
      getPhysicalTarget(index) + delta
    );
  }

  void resetTargetsTowardZero(double max_step)
  {
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
      const double next_target = moveToward(
        getPhysicalTarget(i),
        0.0,
        max_step
      );

      setPhysicalTarget(i, next_target);
    }
  }

  void update()
  {
    const auto now = std::chrono::steady_clock::now();

    double dt =
      std::chrono::duration<double>(now - last_control_time_).count();

    last_control_time_ = now;

    if (dt <= 0.0 || dt > 0.2) {
      dt = 0.02;
    }

    // joint_states를 아직 받지 못했으면 초기 0 rad 명령을 보내지 않음
    if (!target_initialized_) {
      return;
    }

    bool target_changed = false;
    bool force_publish = false;
    bool hold_pressed = false;

    if (joy_received_) {
      hold_pressed = getButton(hold_button_);
      const bool reset_pressed = getButton(reset_button_);

      // A 버튼을 누른 순간: 현재 실제 자세를 목표값으로 저장
      if (hold_pressed && !last_hold_pressed_) {
        setTargetsToCurrentPosition();
        target_changed = true;
        force_publish = true;

        RCLCPP_INFO(
          this->get_logger(),
          "Hold: current positions stored as targets."
        );
      } else if (reset_pressed) {
        resetTargetsTowardZero(angle_rate_ * dt);
        target_changed = true;
      } else {
        const double front_input = getAxis(front_axis_);
        const double rear_input = getAxis(rear_axis_);

        const double front_delta = front_input * angle_rate_ * dt;
        const double rear_delta = rear_input * angle_rate_ * dt;

        if (std::fabs(front_delta) > 1e-8) {
          addPhysicalTarget(0, front_delta);
          addPhysicalTarget(1, front_delta);
          target_changed = true;
        }

        if (std::fabs(rear_delta) > 1e-8) {
          addPhysicalTarget(2, rear_delta);
          addPhysicalTarget(3, rear_delta);
          target_changed = true;
        }
      }
    }

    last_hold_pressed_ = hold_pressed;

    if (target_changed) {
      command_dirty_ = true;
    }

    const double elapsed_since_command =
      std::chrono::duration<double>(
        now - last_command_time_
      ).count();

    // 목표가 바뀐 경우에만 5 Hz로 전송.
    // 기존처럼 매 20 ms마다 trajectory를 교체하지 않는다.
    if (command_dirty_ &&
        (force_publish || elapsed_since_command >= command_period_))
    {
      publishTrajectory();

      command_dirty_ = false;
      last_command_time_ = now;
    }

    const double front_target =
      0.5 * (getPhysicalTarget(0) + getPhysicalTarget(1));

    const double rear_target =
      0.5 * (getPhysicalTarget(2) + getPhysicalTarget(3));

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "target(front=%.3f rear=%.3f) | "
      "raw: FRF %.3f FLF %.3f BRF %.3f BLF %.3f",
      front_target,
      rear_target,
      target_pos_[0],
      target_pos_[1],
      target_pos_[2],
      target_pos_[3]
    );
  }

  void publishTrajectory()
  {
    trajectory_msgs::msg::JointTrajectory cmd;

    // header.stamp를 설정하지 않는다.
    // stamp=0은 controller가 trajectory를 수신 즉시 시작한다는 의미다.
    cmd.joint_names = {
      "FRF_joint",
      "FLF_joint",
      "BRF_joint",
      "BLF_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = {
      target_pos_[0],
      target_pos_[1],
      target_pos_[2],
      target_pos_[3]
    };

    const double safe_duration =
      std::max(0.05, trajectory_duration_);

    const int64_t total_nanoseconds =
      static_cast<int64_t>(
        std::llround(safe_duration * 1e9)
      );

    point.time_from_start.sec =
      static_cast<int32_t>(total_nanoseconds / 1000000000LL);

    point.time_from_start.nanosec =
      static_cast<uint32_t>(total_nanoseconds % 1000000000LL);

    cmd.points.push_back(point);

    cmd_pub_->publish(cmd);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Trajectory sent: [%.3f, %.3f, %.3f, %.3f]",
      target_pos_[0],
      target_pos_[1],
      target_pos_[2],
      target_pos_[3]
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
  double trajectory_duration_;
  double command_period_;

  double frf_sign_;
  double flf_sign_;
  double brf_sign_;
  double blf_sign_;

  bool joy_received_ = false;
  bool target_initialized_ = false;
  bool last_hold_pressed_ = false;
  bool command_dirty_ = false;

  std::array<double, JOINT_COUNT> current_pos_ = {
    0.0, 0.0, 0.0, 0.0
  };

  std::array<bool, JOINT_COUNT> current_pos_received_ = {
    false, false, false, false
  };

  std::array<double, JOINT_COUNT> target_pos_ = {
    0.0, 0.0, 0.0, 0.0
  };

  std::chrono::steady_clock::time_point last_control_time_;
  std::chrono::steady_clock::time_point last_command_time_;

  sensor_msgs::msg::Joy::SharedPtr latest_joy_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlipperJoyControlNode>());
  rclcpp::shutdown();
  return 0;
}
