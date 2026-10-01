// Receives flattened LabVIEW UDP clusters and publishes their ROS 2 messages.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "nissan_bridge_msgs/msg/ev_battery_history.hpp"
#include "nissan_bridge_msgs/msg/ev_battery_power.hpp"
#include "nissan_bridge_msgs/msg/ev_thermal.hpp"
#include "nissan_bridge_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr std::size_t kValueBytes = sizeof(float);
constexpr std::array<std::size_t, 4> kClusterValueCounts{8, 24, 16, 12};
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr int64_t kNanosecondsPerTick = 100LL;

float bytesToFloat(const uint8_t * bytes, bool big_endian_source)
{
  uint32_t raw;
  std::memcpy(&raw, bytes, sizeof(raw));
  if (big_endian_source) {
    raw = __builtin_bswap32(raw);
  }
  float value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

bool toByte(float value, uint8_t & result)
{
  if (!std::isfinite(value) || value < 0.0F || value > 255.0F || std::floor(value) != value) {
    return false;
  }
  result = static_cast<uint8_t>(value);
  return true;
}
}  // namespace

class NissanBridgeNode : public rclcpp::Node
{
public:
  NissanBridgeNode()
  : Node("nissan_bridge_node")
  {
    listen_port_ = declare_parameter<int>("listen_port", 58432);
    source_ip_ = declare_parameter<std::string>("source_ip", "192.162.11.22");
    big_endian_source_ = declare_parameter<bool>("big_endian_source", true);
    time_diff_threshold_sec_ = declare_parameter<double>("time_diff_threshold_sec", 0.5);
    debug_ = declare_parameter<bool>("debug", true);

    battery_power_publisher_ = create_publisher<nissan_bridge_msgs::msg::EvBatteryPower>(
      "ev/battery_power", 10);
    thermal_publisher_ = create_publisher<nissan_bridge_msgs::msg::EvThermal>("ev/thermal", 10);
    battery_history_publisher_ = create_publisher<nissan_bridge_msgs::msg::EvBatteryHistory>(
      "ev/battery_history", 10);
    vehicle_state_publisher_ = create_publisher<nissan_bridge_msgs::msg::VehicleState>(
      "vehicle/state", 10);

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(
              std::string("Failed to create UDP socket: ") + std::strerror(errno));
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    timeval timeout{};
    timeout.tv_sec = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(listen_port_));

    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      const std::string error = std::strerror(errno);
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(
              "Failed to bind UDP socket on port " + std::to_string(listen_port_) + ": " + error);
    }

    RCLCPP_INFO(
      get_logger(), "Listening on 0.0.0.0:%d for LabVIEW UDP packets from %s",
      listen_port_, source_ip_.c_str());

    running_ = true;
    receive_thread_ = std::thread(&NissanBridgeNode::receiveLoop, this);
  }

  ~NissanBridgeNode() override
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void receiveLoop()
  {
    std::array<uint8_t, 65536> buffer{};

    while (running_ && rclcpp::ok()) {
      sockaddr_in sender_address{};
      socklen_t sender_length = sizeof(sender_address);
      const ssize_t received = recvfrom(
        socket_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender_address), &sender_length);

      if (received < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "recvfrom() failed: %s", std::strerror(errno));
        continue;
      }

      char sender_ip[INET_ADDRSTRLEN]{};
      inet_ntop(AF_INET, &sender_address.sin_addr, sender_ip, sizeof(sender_ip));
      const uint16_t sender_port = ntohs(sender_address.sin_port);
      if (source_ip_ != sender_ip) {
        RCLCPP_WARN(
          get_logger(), "Ignoring UDP packet from unexpected source %s:%u (expected %s)",
          sender_ip, sender_port, source_ip_.c_str());
        continue;
      }

      if (static_cast<std::size_t>(received) < kValueBytes ||
        static_cast<std::size_t>(received) % kValueBytes != 0)
      {
        RCLCPP_WARN(
          get_logger(), "Ignoring malformed UDP packet from %s:%u (%zd bytes)",
          sender_ip, sender_port, received);
        continue;
      }

      const std::size_t value_count = static_cast<std::size_t>(received) / kValueBytes;
      std::vector<float> values(value_count);
      for (std::size_t index = 0; index < value_count; ++index) {
        values[index] = bytesToFloat(buffer.data() + index * kValueBytes, big_endian_source_);
      }

      const float cluster_id = values[0];
      if (!std::isfinite(cluster_id) || std::floor(cluster_id) != cluster_id ||
        cluster_id < 1.0F || cluster_id > 4.0F)
      {
        RCLCPP_WARN(get_logger(), "Ignoring packet with invalid cluster ID %.3f", cluster_id);
        continue;
      }

      const std::size_t expected_count = kClusterValueCounts[static_cast<std::size_t>(cluster_id) - 1];
      if (value_count != expected_count) {
        RCLCPP_WARN(
          get_logger(), "Ignoring cluster %.0f packet with %zu floats; expected %zu",
          cluster_id, value_count, expected_count);
        continue;
      }

      const double time_diff_sec = checkTimeDifference(
        static_cast<std::size_t>(cluster_id) - 1, values[1]);
      if (debug_) {
        std::ostringstream packet_log;
        packet_log << std::fixed << std::setprecision(6)
                   << "Cluster " << cluster_id << " (" << received << " bytes), time diff="
                   << time_diff_sec << " sec:";
        for (std::size_t index = 0; index < values.size(); ++index) {
          packet_log << " [" << index << "]=" << values[index];
        }
        RCLCPP_INFO(get_logger(), "%s", packet_log.str().c_str());
      }

      switch (static_cast<unsigned int>(cluster_id)) {
        case 1:
          publishBatteryPower(values);
          break;
        case 2:
          publishThermal(values);
          break;
        case 3:
          publishBatteryHistory(values);
          break;
        case 4:
          publishVehicleState(values);
          break;
        default:
          break;
      }
    }
  }

  double checkTimeDifference(std::size_t cluster_index, float source_ticks)
  {
    if (!std::isfinite(source_ticks) || source_ticks < 0.0F) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid source timestamp %.3f", source_ticks);
      return std::numeric_limits<double>::quiet_NaN();
    }

    const int64_t ros_time_ns = now().nanoseconds();
    const int64_t source_ticks_ns = std::llround(
      static_cast<double>(source_ticks) * static_cast<double>(kNanosecondsPerTick));

    if (!time_baseline_set_[cluster_index] ||
      source_ticks_ns < first_source_time_ns_[cluster_index])
    {
      time_baseline_set_[cluster_index] = true;
      first_source_time_ns_[cluster_index] = source_ticks_ns;
      first_ros_time_ns_[cluster_index] = ros_time_ns;
      RCLCPP_INFO(
        get_logger(),
        "Cluster %zu initial ROS/CAN elapsed-time diff: 0.000000 sec (baseline established)",
        cluster_index + 1);
      return 0.0;
    }

    const int64_t ros_elapsed_ns = ros_time_ns - first_ros_time_ns_[cluster_index];
    const int64_t source_elapsed_ns = source_ticks_ns - first_source_time_ns_[cluster_index];
    const double diff_sec = static_cast<double>(ros_elapsed_ns - source_elapsed_ns) /
      static_cast<double>(kNanosecondsPerSecond);

    if (std::fabs(diff_sec) > time_diff_threshold_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cluster %zu ROS/CAN elapsed-time difference is %.6f sec (threshold %.3f sec)",
        cluster_index + 1, diff_sec, time_diff_threshold_sec_);
    }
      return diff_sec;
  }

  void publishBatteryPower(const std::vector<float> & values)
  {
    nissan_bridge_msgs::msg::EvBatteryPower message;
    message.header.stamp = now();
    message.ev_1db_timestamp = values[1];
    message.ev_1db_lb_voltage = values[2];
    message.ev_1db_lb_current = values[3];
    message.ev_1db_lb_usable_soc = values[4];
    message.ev_1dc_timestamp = values[5];
    message.ev_1dc_lb_charge_power_limit = values[6];
    message.ev_1dc_lb_discharge_power_limit = values[7];
    battery_power_publisher_->publish(message);
  }

  void publishThermal(const std::vector<float> & values)
  {
    nissan_bridge_msgs::msg::EvThermal message;
    message.header.stamp = now();
    message.ev_54a_timestamp = values[1];
    message.ev_54a_ambient_temp_ac = values[2];
    message.ev_54c_timestamp = values[3];
    message.ev_54c_ac_evaporator_temperature = values[4];
    message.ev_54f_timestamp = values[5];
    message.ev_54f_interior_intake_temp = values[6];
    message.ev_55a_timestamp = values[7];
    message.ev_55a_motor_temperature = values[8];
    message.ev_55a_igbt_temperature = values[9];
    message.ev_55a_igbt_driver_board_temperature = values[10];
    message.ev_55a_inverter_com_board_temp = values[11];
    message.ev_55b_timestamp = values[12];
    message.ev_55b_lb_soc = values[13];
    message.ev_5bc_timestamp = values[14];
    message.ev_5bc_lb_capacity_deterioration_rate = values[15];
    message.ev_5bc_lb_remain_capacity_gids = values[16];
    message.ev_5bc_lb_temperature_segment_for_dash = values[17];
    message.car_510_timestamp = values[18];
    message.car_510_outside_ambient_temperature = values[19];
    message.car_5b3_timestamp = values[20];
    message.car_5b3_battery_state_of_health = values[21];
    message.car_5b3_battery_gids = values[22];
    message.car_5b3_battery_pack_temperature = values[23];
    thermal_publisher_->publish(message);
  }

  void publishBatteryHistory(const std::vector<float> & values)
  {
    nissan_bridge_msgs::msg::EvBatteryHistory message;
    message.header.stamp = now();
    message.ev_59e_timestamp = values[1];
    message.ev_59e_lb_full_capacity_for_qc = values[2];
    message.ev_5c0_timestamp = values[3];
    message.ev_5c0_degr_int_res_coeff_min = values[4];
    message.ev_5c0_degr_int_res_coeff_max = values[5];
    message.ev_5c0_degr_int_res_coeff_avg = values[6];
    message.ev_5c0_cell_voltage_min = values[7];
    message.ev_5c0_cell_voltage_max = values[8];
    message.ev_5c0_cell_voltage_avg = values[9];
    message.ev_5c0_temperature_min = values[10];
    message.ev_5c0_temperature_max = values[11];
    message.ev_5c0_temperature_avg = values[12];
    message.ev_5c0_temp_wakeup_phase_min = values[13];
    message.ev_5c0_temp_wakeup_phase_max = values[14];
    message.ev_5c0_temp_wakeup_phase_avg = values[15];
    battery_history_publisher_->publish(message);
  }

  void publishVehicleState(const std::vector<float> & values)
  {
    nissan_bridge_msgs::msg::VehicleState message;
    message.header.stamp = now();
    message.rt_time = values[1];
    message.aut_time = values[2];
    message.car_284_timestamp = values[3];
    message.car_284_vehicle_speed_from_abs = values[4];
    message.car_002_timestamp = values[5];
    message.car_002_steering_angle = values[6];
    message.accel_pedal_state = values[7];
    message.brake_pedal_state = values[8];
    if (!toByte(values[9], message.control_state) ||
      !toByte(values[11], message.car_358_turn_signal_status))
    {
      RCLCPP_WARN(get_logger(), "Ignoring vehicle state packet with invalid byte field");
      return;
    }
    message.car_358_timestamp = values[10];
    vehicle_state_publisher_->publish(message);
  }

  int listen_port_{58432};
  std::string source_ip_{"192.162.11.22"};
  bool big_endian_source_{true};
  double time_diff_threshold_sec_{0.5};
  bool debug_{false};
  int socket_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  std::array<bool, 4> time_baseline_set_{};
  std::array<int64_t, 4> first_source_time_ns_{};
  std::array<int64_t, 4> first_ros_time_ns_{};
  rclcpp::Publisher<nissan_bridge_msgs::msg::EvBatteryPower>::SharedPtr battery_power_publisher_;
  rclcpp::Publisher<nissan_bridge_msgs::msg::EvThermal>::SharedPtr thermal_publisher_;
  rclcpp::Publisher<nissan_bridge_msgs::msg::EvBatteryHistory>::SharedPtr
    battery_history_publisher_;
  rclcpp::Publisher<nissan_bridge_msgs::msg::VehicleState>::SharedPtr vehicle_state_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<NissanBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("nissan_bridge_node"), "Fatal error: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
