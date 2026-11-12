// Longitudinal (speed) PID control for the Nissan, ported from the Lexus speed_control node.
//
// Reference: cmd_vel (geometry_msgs/Twist, linear.x in m/s)
// Feedback:  vehicle_status (geometry_msgs/TwistStamped from nissan_bridge, linear.x in m/s)
// Output:    control/accel_cmd, control/brake_cmd (nissan_bridge_msgs/Float64Stamped, 0..1)
//
// Steering is not handled here: nissan_bridge forwards cmd_vel angular.z to the cRIO directly.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nissan_bridge_msgs/msg/float64_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using Float64Stamped = nissan_bridge_msgs::msg::Float64Stamped;

class SpeedControl : public rclcpp::Node
{
public:
  SpeedControl()
  : Node("speed_control_node")
  {
    p_gain_accel_ = declare_parameter<double>("p_gain_accel", 0.29);
    i_gain_accel_ = declare_parameter<double>("i_gain_accel", 0.035);
    d_gain_accel_ = declare_parameter<double>("d_gain_accel", 0.0);
    p_gain_brake_ = declare_parameter<double>("p_gain_brake", 0.15);
    i_gain_brake_ = declare_parameter<double>("i_gain_brake", 0.038);
    d_gain_brake_ = declare_parameter<double>("d_gain_brake", 0.0);
    // Anti-windup limits of the controller output; the cRIO expects 0..1
    // (Lexus used max_brake 1.5)
    max_accel_ = declare_parameter<double>("max_accel", 1.0);
    max_brake_ = declare_parameter<double>("max_brake", 1.0);
    // Rate limits, output units per second
    accel_rate_up_ = declare_parameter<double>("accel_rate_up", 0.4);
    accel_rate_down_ = declare_parameter<double>("accel_rate_down", 1.6);
    brake_rate_up_ = declare_parameter<double>("brake_rate_up", 0.8);
    // Switching hysteresis between accel and brake mode, m/s
    hysteresis_ = declare_parameter<double>("hysteresis", 0.9);
    standstill_brake_ = declare_parameter<double>("standstill_brake", 0.3);
    // CMD_TIMEOUT (disabled for now): without cmd_vel for this long the reference falls back
    // to 0 m/s (brake to standstill). Uncomment all CMD_TIMEOUT blocks to enable it again.
    // cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    // dt used when the measured feedback period is missing or implausible
    nominal_dt_ = declare_parameter<double>("nominal_dt", 0.01);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    speed_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "vehicle_status", 10, std::bind(&SpeedControl::speedCurrentCallback, this, std::placeholders::_1));
    reference_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&SpeedControl::speedReferenceCallback, this, std::placeholders::_1));
    reinit_sub_ = create_subscription<std_msgs::msg::Bool>(
      "control_reinit", 10, std::bind(&SpeedControl::reinitCallback, this, std::placeholders::_1));

    accel_pub_ = create_publisher<Float64Stamped>("control/accel_cmd", 10);
    brake_pub_ = create_publisher<Float64Stamped>("control/brake_cmd", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("control_status", 10);

    RCLCPP_INFO_STREAM(
      get_logger(), "Starting longitudinal (speed) control. Namespace: " << get_namespace());
    RCLCPP_INFO_STREAM(
      get_logger(), "Accel PID: " << p_gain_accel_ << " | " << i_gain_accel_ << " | " << d_gain_accel_);
    RCLCPP_INFO_STREAM(
      get_logger(), "Brake PID: " << p_gain_brake_ << " | " << i_gain_brake_ << " | " << d_gain_brake_);
  }

private:
  void speedCurrentCallback(const geometry_msgs::msg::TwistStamped & speed_msg)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    double dt = nominal_dt_;
    if (last_feedback_time_) {
      const double measured = std::chrono::duration<double>(now_steady - *last_feedback_time_).count();
      if (measured > 1e-4 && measured < 0.5) {
        dt = measured;
      }
    }
    last_feedback_time_ = now_steady;

    const double speed_actual = speed_msg.twist.linear.x;   // m/s
    const double speed_reference = speed_reference_;
    // CMD_TIMEOUT (disabled for now), also drop the const from speed_reference above:
    // const bool reference_stale = !last_reference_time_ ||
    //   std::chrono::duration<double>(now_steady - *last_reference_time_).count() > cmd_timeout_sec_;
    // if (reference_stale) {
    //   speed_reference = 0.0;
    //   RCLCPP_WARN_THROTTLE(
    //     get_logger(), *get_clock(), 2000,
    //     "No cmd_vel for more than %.2f sec, speed reference set to 0", cmd_timeout_sec_);
    // }

    const double speed_diff = speed_reference - speed_actual;
    const double derivative = first_run_ ? 0.0 : (speed_diff - speed_diff_prev_) / dt;
    std::string status;

    // Hysteresis between accel and brake mode avoids fluctuating at constant speed
    if ((speed_diff > -hysteresis_ && accel_mode_) || speed_diff > hysteresis_) {
      if (!accel_mode_) {
        integ_brake_ = 0.0;
      }
      accel_mode_ = true;
      status = "accel";

      const double p_out = p_gain_accel_ * speed_diff;
      integ_accel_ += speed_diff * dt;
      const double d_out = d_gain_accel_ * derivative;
      double command = p_out + i_gain_accel_ * integ_accel_ + d_out;
      if (command > max_accel_) {  // anti-windup
        command = max_accel_;
        if (i_gain_accel_ > 0.0) {
          integ_accel_ = (max_accel_ - p_out - d_out) / i_gain_accel_;
        }
      }
      command = std::max(0.0, command);
      // Gradient limit
      command = std::clamp(
        command, accel_command_prev_ - accel_rate_down_ * dt, accel_command_prev_ + accel_rate_up_ * dt);
      accel_command_ = command;
      brake_command_ = 0.0;
    } else {
      if (accel_mode_) {
        integ_accel_ = 0.0;
      }
      accel_mode_ = false;
      status = "brake";

      // Brake effort is positive when the vehicle is too fast (speed_diff < 0)
      double p_out = -p_gain_brake_ * speed_diff;
      // Standstill 1.38 m/s ~= 5.0 km/h
      if (speed_reference < 1.38 && speed_actual < 1.38) {
        status = "standstill5";
        p_out *= 1.2;
      }
      integ_brake_ -= speed_diff * dt;
      const double d_out = -d_gain_brake_ * derivative;
      double command = p_out + i_gain_brake_ * integ_brake_ + d_out;
      if (command > max_brake_) {  // anti-windup
        command = max_brake_;
        if (i_gain_brake_ > 0.0) {
          integ_brake_ = (max_brake_ - p_out - d_out) / i_gain_brake_;
        }
      }
      command = std::max(0.0, command);
      // Gradient limit (only when increasing the brake)
      command = std::min(command, brake_command_prev_ + brake_rate_up_ * dt);
      // Standstill 0.27 m/s ~= 1.0 km/h: hold the vehicle
      if (speed_reference < 0.01 && speed_actual < 0.27) {
        status = "standstill1";
        command = standstill_brake_;
      }
      brake_command_ = command;
      accel_command_ = 0.0;
    }

    if (first_run_) {
      first_run_ = false;
      status = "longitudinal_control_started";
    }

    const auto stamp = now();
    publishCommand(accel_pub_, stamp, accel_command_);
    publishCommand(brake_pub_, stamp, brake_command_);
    std_msgs::msg::String status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);

    accel_command_prev_ = accel_command_;
    brake_command_prev_ = brake_command_;
    speed_diff_prev_ = speed_diff;
  }

  void speedReferenceCallback(const geometry_msgs::msg::Twist & ref_msg)
  {
    speed_reference_ = ref_msg.linear.x;
    last_reference_time_ = std::chrono::steady_clock::now();
  }

  void reinitCallback(const std_msgs::msg::Bool & msg)
  {
    // ros2 topic pub --once /nissan9/control_reinit std_msgs/msg/Bool "{data: true}"
    RCLCPP_INFO_STREAM(
      get_logger(), get_namespace() << "/control_reinit: " << (msg.data ? "true" : "false"));
    if (!msg.data) {
      return;
    }
    integ_accel_ = integ_brake_ = 0.0;
    accel_command_prev_ = brake_command_prev_ = 0.0;
    speed_diff_prev_ = 0.0;
    accel_mode_ = true;
    first_run_ = true;
  }

  void publishCommand(
    const rclcpp::Publisher<Float64Stamped>::SharedPtr & publisher,
    const rclcpp::Time & stamp, double value)
  {
    Float64Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.data = value;
    publisher->publish(msg);
  }

  double p_gain_accel_, i_gain_accel_, d_gain_accel_;
  double p_gain_brake_, i_gain_brake_, d_gain_brake_;
  double max_accel_, max_brake_;
  double accel_rate_up_, accel_rate_down_, brake_rate_up_;
  double hysteresis_, standstill_brake_;
  // double cmd_timeout_sec_;   // CMD_TIMEOUT (disabled for now)
  double nominal_dt_;
  std::string frame_id_;

  double speed_reference_{0.0};
  double speed_diff_prev_{0.0};
  double integ_accel_{0.0}, integ_brake_{0.0};
  double accel_command_{0.0}, brake_command_{0.0};
  double accel_command_prev_{0.0}, brake_command_prev_{0.0};
  bool accel_mode_{true};  // accel (true) / brake (false) mode, needed for the hysteresis
  bool first_run_{true};
  std::optional<std::chrono::steady_clock::time_point> last_feedback_time_;
  std::optional<std::chrono::steady_clock::time_point> last_reference_time_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reinit_sub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr brake_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedControl>());
  rclcpp::shutdown();
  return 0;
}
