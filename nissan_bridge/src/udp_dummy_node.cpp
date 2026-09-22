// Dummy UDP receiver node.
//
// Listens for UDP packets sent from a LabVIEW target (default 10.0.0.3:58432)
// on a local UDP port (default 58432). Each packet is expected to contain a
// flattened array of 54 doubles (64-bit IEEE-754, 432 bytes total). Elements
// 32, 33, 34 and 35 (0-indexed) are printed to the terminal on every packet.
//
// This node exists purely to validate the wire format coming from LabVIEW; it
// does not publish anything onto the ROS graph.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr std::size_t kArraySize = 54;
constexpr std::size_t kPacketBytes = kArraySize * sizeof(double);

// LabVIEW's "Flatten To String" defaults to big-endian (network) byte order,
// while x86/x64 Linux is little-endian, so the bytes of every double need to
// be swapped before they can be reinterpreted. This assumes a little-endian
// host, which holds for the platforms this bridge targets.
double bytesToDouble(const uint8_t * bytes, bool big_endian_source)
{
  uint64_t raw;
  std::memcpy(&raw, bytes, sizeof(raw));
  if (big_endian_source) {
    raw = __builtin_bswap64(raw);
  }
  double value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}
}  // namespace

class UdpDummyNode : public rclcpp::Node
{
public:
  UdpDummyNode()
  : Node("udp_dummy_node")
  {
    listen_port_ = declare_parameter<int>("listen_port", 58432);
    source_ip_ = declare_parameter<std::string>("source_ip", "10.0.0.3");
    big_endian_source_ = declare_parameter<bool>("big_endian_source", true);

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(
              std::string("Failed to create UDP socket: ") + std::strerror(errno));
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Receive timeout so the worker thread can periodically check rclcpp::ok()
    // and shut down cleanly instead of blocking forever in recvfrom().
    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(listen_port_));

    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      const std::string err = std::strerror(errno);
      close(socket_fd_);
      throw std::runtime_error(
              "Failed to bind UDP socket on port " + std::to_string(listen_port_) + ": " + err);
    }

    RCLCPP_INFO(
      get_logger(),
      "udp_dummy_node listening on 0.0.0.0:%d, expecting %zu-double packets from %s",
      listen_port_, kArraySize, source_ip_.c_str());

    running_ = true;
    recv_thread_ = std::thread(&UdpDummyNode::receiveLoop, this);
  }

  ~UdpDummyNode() override
  {
    running_ = false;
    if (recv_thread_.joinable()) {
      recv_thread_.join();
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
      sockaddr_in sender_addr{};
      socklen_t sender_len = sizeof(sender_addr);

      ssize_t received = recvfrom(
        socket_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender_addr), &sender_len);

      if (received < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
          continue;  // recv timeout, loop back and re-check running_/rclcpp::ok()
        }
        RCLCPP_ERROR(get_logger(), "recvfrom() failed: %s", std::strerror(errno));
        continue;
      }

      char sender_ip[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, sizeof(sender_ip));
      const uint16_t sender_port = ntohs(sender_addr.sin_port);

      if (source_ip_ != sender_ip) {
        RCLCPP_WARN(
          get_logger(), "Ignoring UDP packet from unexpected source %s:%u (expected %s)",
          sender_ip, sender_port, source_ip_.c_str());
        continue;
      }

      if (static_cast<std::size_t>(received) != kPacketBytes) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring packet from %s:%u with unexpected size: got %zd bytes, expected %zu "
          "bytes (%zu doubles)",
          sender_ip, sender_port, received, kPacketBytes, kArraySize);
        continue;
      }

      std::array<double, kArraySize> values{};
      for (std::size_t i = 0; i < kArraySize; ++i) {
        values[i] = bytesToDouble(buffer.data() + i * sizeof(double), big_endian_source_);
      }

      RCLCPP_INFO(
        get_logger(), "[%s:%u] [32]=%.6f [33]=%.6f [34]=%.6f [35]=%.6f",
        sender_ip, sender_port, values[32], values[33], values[34], values[35]);
    }
  }

  int listen_port_{58432};
  std::string source_ip_{"10.0.0.3"};
  bool big_endian_source_{true};
  int socket_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread recv_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UdpDummyNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("udp_dummy_node"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
