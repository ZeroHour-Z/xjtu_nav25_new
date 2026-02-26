#include "packet_utils.hpp"
#include "protocol.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
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

// 本节点直接操作串口设备：
// - 参数：port=/dev/ttyACM0, baud=115200
// - 订阅: /rm_comm/tx_packet (handler 打包好的 64 字节，std_msgs/String)
// - 发布: /rm_comm/rx_packet (从串口读取的原始数据，std_msgs/String)

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

class SerialRwNode: public rclcpp::Node {
public:
    SerialRwNode(): Node("serial_rw_node") {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baud_ = this->declare_parameter<int>("baud", 115200);
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

        tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/tx_packet",
            rclcpp::QoS(10).reliable(),
            std::bind(&SerialRwNode::onTxPacket, this, std::placeholders::_1)
        );

        rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/rx_packet",
            rclcpp::QoS(10).reliable()
        );
        running_.store(true);

        read_thread_ = std::thread(&SerialRwNode::readLoop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "serial_rw_node started, port:%s, baud:%d",
            port_.c_str(),
            baud_
        );
    }

    ~SerialRwNode() override {
        running_.store(false);
        closeSerial();
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

private:
    void onTxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        // 严格校验：仅允许 navInfo_t 长度
        constexpr size_t kTxPacketSize = sizeof(navInfo_t);
        if (msg->data.size() != kTxPacketSize) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "tx_packet size %zu != %zu, drop",
                msg->data.size(),
                kTxPacketSize
            );
            return;
        }

        const int fd_snapshot = currentFd();
        if (fd_snapshot < 0) {
            // 触发重连由读线程负责，此处提示
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "serial not open; drop tx"
            );
            return;
        }

        const uint8_t* data = reinterpret_cast<const uint8_t*>(msg->data.data());
        size_t remaining = msg->data.size();
        while (remaining > 0) {
            ssize_t n = ::write(fd_snapshot, data, remaining);
            if (n < 0) {
                if (errno == EINTR)
                    continue;
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
                rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);

                // 常量：仅接受 0x72 开头、0x21 结尾的 navCommand_t 帧
                constexpr uint8_t kHeader = 0x72;
                constexpr uint8_t kTailCommand = 0x21;
                constexpr size_t kPktSize = sizeof(navCommand_t); // 64

                // 分帧与重同步
                while (true) {
                    // 找到下一次帧头
                    auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), kHeader);
                    if (it == rx_buffer_.end()) {
                        // 没有帧头，清空缓冲
                        rx_buffer_.clear();
                        break;
                    }
                    // 丢弃帧头之前的垃圾数据
                    if (it != rx_buffer_.begin()) {
                        rx_buffer_.erase(rx_buffer_.begin(), it);
                    }
                    // 等待完整帧
                    if (rx_buffer_.size() < kPktSize)
                        break;

                    // 检查帧尾
                    if (rx_buffer_[kPktSize - 1] != kTailCommand) {
                        // 非法帧，丢弃当前帧头，继续向后搜索
                        rx_buffer_.erase(rx_buffer_.begin());
                        continue;
                    }

                    // 提取并发布
                    navCommand_t n_data;
                    std::memcpy(&n_data, rx_buffer_.data(), kPktSize);

                    // 打印从电控收到的数据（tail:0x21）
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "RX -> Color:%d State:%d HP:%d Bullet:%d Enemy_x:%f Enemy_y:%f is_revive:%d Target_x:%f Target_y:%f",
                        (int)n_data.color,
                        (int)n_data.eSentryState,
                        (int)n_data.hp_remain,
                        (int)n_data.bullet_remain,
                        n_data.enemy_x,
                        n_data.enemy_y,
                        (int)n_data.is_revive,
                        n_data.target_x,
                        n_data.target_y
                    );

                    std_msgs::msg::UInt8MultiArray out_msg;
                    const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(&n_data);
                    out_msg.data.assign(byte_ptr, byte_ptr + kPktSize);
                    rx_pub_->publish(out_msg);

                    // 移除已消费帧
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kPktSize);
                }

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
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "open %s failed: %s",
                port_.c_str(),
                std::strerror(errno)
            );
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
        tio.c_cflag &= ~CSTOPB; // 1 个停止位
        tio.c_cflag &= ~PARENB; // 无校验
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8; // 8 位

        speed_t spd = to_baud_constant(baud_);
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);

        // 读阻塞条件：至少 1 字节即可返回
        tio.c_cc[VMIN] = 1;
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
        RCLCPP_INFO(this->get_logger(), "Serial opened");
        return true;
    }

    void closeSerial() {
        int fd_to_close = -1;
        {
            std::lock_guard<std::mutex> lock(fd_mutex_);
            if (fd_ >= 0) {
                fd_to_close = fd_;
                fd_ = -1;
            }
        }
        if (fd_to_close >= 0) {
            ::close(fd_to_close);
            RCLCPP_INFO(this->get_logger(), "Serial closed");
        }
    }

    // members
    std::string port_;
    int baud_ { 115200 };
    int reopen_interval_ms_ { 500 };
    std::chrono::milliseconds idle_sleep_duration_ { 1 };

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_pub_;

    std::thread read_thread_;
    std::atomic<bool> running_ { false };
    mutable std::mutex fd_mutex_;
    int fd_ { -1 };
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
