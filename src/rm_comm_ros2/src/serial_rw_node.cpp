#include "packet_utils.hpp"
#include "protocol.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

// 本节点直接操作串口设备，支持同时处理自瞄和导航两个系统的数据：
// - 参数：port=/dev/ttyACM0, baud=115200
// - 参数：enable_aim=false (设为 true 可同时处理自瞄系统数据)
// - 导航系统：
//   * 订阅: /rm_comm/tx_packet (导航节点打包好的 64 字节，帧头 0x72，帧尾 0x21/0x4D)
//   * 发布: /rm_comm/rx_packet (从串口读取的导航数据)
// - 自瞄系统（当 enable_aim=true 时）：
//   * 订阅: /autoaim/tx (视觉节点打包好的 64 字节，帧头 0x71，帧尾 0x4C)
//   * 发布: /autoaim/rx (从串口读取的自瞄数据)
// - 接收数据时会根据帧头/帧尾自动识别数据来源并发布到对应的topic

namespace {

  static speed_t to_baud_constant(int baud) {
    switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
    }
  }

} // namespace

class SerialRwNode : public rclcpp::Node {
public:
  SerialRwNode() : Node("serial_rw_node") {
    port_               = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_               = this->declare_parameter<int>("baud", 115200);
    reopen_interval_ms_ = this->declare_parameter<int>("reopen_interval_ms", 500);
    double read_loop_hz = this->declare_parameter<double>("read_loop_hz", 200.0);
    if (read_loop_hz <= 0.0) {
      read_loop_hz = 1.0;
    }
    auto desired_period = std::chrono::duration<double>(1.0 / read_loop_hz);
    idle_sleep_duration_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(desired_period);
    if (idle_sleep_duration_.count() <= 0) {
      idle_sleep_duration_ = std::chrono::milliseconds(1);
    }

    // 支持同时处理自瞄和导航两个系统的数据
    enable_aim_ = this->declare_parameter<bool>("enable_aim", false);

    // 订阅导航系统的发送topic
    tx_nav_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/rm_comm/tx_packet", rclcpp::QoS(10).reliable(),
        std::bind(&SerialRwNode::onTxNavPacket, this, std::placeholders::_1));

    // 发布到导航系统的接收topic
    rx_nav_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/rm_comm/rx_packet", rclcpp::QoS(10).reliable());
    
    // 如果启用自瞄系统，也订阅和发布自瞄系统的topic
    if (enable_aim_) {
      tx_aim_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/autoaim/tx", 10,
        std::bind(&SerialRwNode::onTxAimPacket, this, std::placeholders::_1));
      
      rx_aim_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/autoaim/rx", 10);
      
      RCLCPP_INFO(this->get_logger(), "Aim system enabled, will handle both aim and nav data");
    }
    
    running_.store(true);

    read_thread_ = std::thread(&SerialRwNode::readLoop, this);

    RCLCPP_INFO(this->get_logger(), "serial_rw_node started. Using port %s, baud %d",
                port_.c_str(), baud_);
  }

  ~SerialRwNode() override {
    running_.store(false);
    closeSerial();
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
  }

private:
  void onTxNavPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // 严格校验：仅允许 navInfo_t 长度
    constexpr size_t kTxPacketSize = sizeof(navInfo_t);
    if (msg->data.size() != kTxPacketSize) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tx_packet size %zu != %zu, drop", msg->data.size(), kTxPacketSize);
      return;
    }
    writeToSerial(msg);
  }
  
  void onTxAimPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // 严格校验：仅允许 64 字节
    constexpr size_t kPacketSize = 64;
    if (msg->data.size() != kPacketSize) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tx_aim_packet size %zu != %zu, drop", msg->data.size(), kPacketSize);
      return;
    }
    writeToSerial(msg);
  }
  
  void writeToSerial(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    const int fd_snapshot = currentFd();
    if (fd_snapshot < 0) {
      // 触发重连由读线程负责，此处提示
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "serial not open; drop tx");
      return;
    }

    const uint8_t* data      = reinterpret_cast<const uint8_t*>(msg->data.data());
    size_t         remaining = msg->data.size();
    while (remaining > 0) {
      ssize_t n = ::write(fd_snapshot, data, remaining);
      if (n < 0) {
        if (errno == EINTR) continue;
        RCLCPP_ERROR(this->get_logger(), "write error: %s", std::strerror(errno));
        closeSerial();
        return;
      }
      remaining -= static_cast<size_t>(n);
      data += n;
    }
  }

  void readLoop() {
    while (running_.load()) {
      int fd_snapshot = currentFd();
      if (fd_snapshot < 0) {
        if (!openSerial()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(reopen_interval_ms_));
          continue;
        }
        continue;
      }

      // 读取串口数据并累积到缓冲区，进行定长分帧与严格校验
      uint8_t buf[512];
      ssize_t n = ::read(fd_snapshot, buf, sizeof(buf));
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "read n: %ld", n);

      if (n > 0) {
        handleIncomingBytes(buf, static_cast<size_t>(n));
        continue;
      }

      if (n == 0) {
        // 无数据，稍作等待
        std::this_thread::sleep_for(idle_sleep_duration_);
        continue;
      }

      // n < 0
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(idle_sleep_duration_);
        continue;
      }

      RCLCPP_WARN(this->get_logger(), "read error: %s", std::strerror(errno));
      closeSerial();
      std::this_thread::sleep_for(std::chrono::milliseconds(reopen_interval_ms_));
    }
  }

  bool openSerial() {
    closeSerial();
    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "open %s failed: %s",
                            port_.c_str(), std::strerror(errno));
      return false;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }

    // 配置 8N1，无流控，原始模式
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS; // 无硬件流控
    tio.c_cflag &= ~CSTOPB;  // 1 个停止位
    tio.c_cflag &= ~PARENB;  // 无校验
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8; // 8 位

    speed_t spd = to_baud_constant(baud_);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // 读阻塞条件：至少 1 字节即可返回
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 1; // 0.1s 单位，设置 1 即 ~100ms 超时

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }

    // 设为阻塞模式，结合 VMIN/VTIME 控制
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      fd_ = fd;
    }
    RCLCPP_INFO(this->get_logger(), "Opened %s @ %d", port_.c_str(), baud_);
    return true;
  }

  void closeSerial() {
    int fd_to_close = -1;
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      if (fd_ >= 0) {
        fd_to_close = fd_;
        fd_         = -1;
      }
    }
    if (fd_to_close >= 0) {
      ::close(fd_to_close);
      RCLCPP_INFO(this->get_logger(), "Serial closed");
    }
  }

  void handleIncomingBytes(const uint8_t *data, size_t n) {
    // 缓存数据
    rx_buffer_.insert(rx_buffer_.end(), data, data + n);
    constexpr size_t kPacketSize = 64;
    
    // 协议定义：
    // 自瞄系统：帧头 0x71，帧尾 0x4C
    // 导航系统：帧头 0x72，帧尾 0x21 (marketCommand_t) 或 0x4D (navInfo_t)
    
    while (rx_buffer_.size() >= kPacketSize) {
      // 尝试识别帧类型
      uint8_t header = rx_buffer_[0];
      uint8_t tail = rx_buffer_[kPacketSize - 1];
      
      bool is_valid_frame = false;
      bool is_aim_frame = false;
      bool is_nav_frame = false;
      
      // 检查是否为自瞄系统帧 (0x71 开头，0x4C 结尾)
      if (header == 0x71 && tail == 0x4C) {
        is_valid_frame = true;
        is_aim_frame = true;
      }
      // 检查是否为导航系统帧 (0x72 开头，0x21 或 0x4D 结尾)
      else if (header == 0x72 && (tail == 0x21 || tail == 0x4D)) {
        is_valid_frame = true;
        is_nav_frame = true;
      }
      
      if (is_valid_frame) {
        // 提取并发布到对应的topic
        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data.assign(rx_buffer_.begin(), rx_buffer_.begin() + kPacketSize);
        
        if (is_aim_frame && enable_aim_) {
          rx_aim_pub_->publish(out_msg);
        } else if (is_nav_frame) {
          rx_nav_pub_->publish(out_msg);
        }
        
        // 移除已消费帧
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kPacketSize);
      } else {
        // 非法帧，尝试找到下一个可能的帧头
        auto it_71 = std::find(rx_buffer_.begin() + 1, rx_buffer_.end(), 0x71);
        auto it_72 = std::find(rx_buffer_.begin() + 1, rx_buffer_.end(), 0x72);
        
        auto next_header = rx_buffer_.end();
        if (it_71 != rx_buffer_.end() && it_72 != rx_buffer_.end()) {
          next_header = (it_71 < it_72) ? it_71 : it_72;
        } else if (it_71 != rx_buffer_.end()) {
          next_header = it_71;
        } else if (it_72 != rx_buffer_.end()) {
          next_header = it_72;
        }
        
        if (next_header != rx_buffer_.end()) {
          // 丢弃到下一个帧头之前的数据
          rx_buffer_.erase(rx_buffer_.begin(), next_header);
        } else {
          // 没有找到有效帧头，清空缓冲
          rx_buffer_.clear();
          break;
        }
      }
    }
  }

  // members
  std::string port_;
  int         baud_{115200};
  int         reopen_interval_ms_{500};
  std::chrono::milliseconds idle_sleep_duration_{1};
  bool enable_aim_{false};

  // 导航系统的订阅和发布
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_nav_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr    rx_nav_pub_;
  
  // 自瞄系统的订阅和发布（可选）
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_aim_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr    rx_aim_pub_;

  std::thread       read_thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex fd_mutex_;
  int               fd_{-1};
  std::vector<uint8_t> rx_buffer_;

  int currentFd() const {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    return fd_;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialRwNode>());
  rclcpp::shutdown();
  return 0;
}
