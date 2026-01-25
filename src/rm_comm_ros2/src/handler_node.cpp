#include "protocol.h"
#include "rm_comm_ros2/packet_utils.hpp"

#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <vector>

class HandlerNode : public rclcpp::Node {
public:
  HandlerNode() : Node("handler_node") {
    patrol_group_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol_group", 10);

    tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rm_comm/tx_packet", 10);

    rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/rm_comm/rx_packet", 100,
        std::bind(&HandlerNode::onRxPacket, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&HandlerNode::onCmdVel, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&HandlerNode::onOdom, this, std::placeholders::_1));

    // 订阅 /goal_pose 用于接收 RViz 2D Goal Pose
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", rclcpp::QoS(10).best_effort(),
        std::bind(&HandlerNode::onGoalPose, this, std::placeholders::_1));

    // 只声明一次参数，避免重复声明异常
    this->declare_parameter<double>("tx_hz", 100.0);
    this->declare_parameter<double>("target_x", 0.0);
    this->declare_parameter<double>("target_y", 0.0);

    double hz = 100.0;
    this->get_parameter("tx_hz", hz);
    tx_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz))),
        std::bind(&HandlerNode::publishTxPacket, this));

    RCLCPP_INFO(this->get_logger(), "handler_node started");
  }

  ~HandlerNode() override = default;

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(),"cmd_vel:%f",msg->linear.x);
    nav_info_.x_speed     = msg->linear.x;
    nav_info_.y_speed     = msg->linear.y;
    nav_info_.yaw_current = msg->angular.x;
    nav_info_.yaw_desired = msg->angular.z;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    nav_info_.x_current = static_cast<float>(msg->pose.pose.position.x);
    nav_info_.y_current = static_cast<float>(msg->pose.pose.position.y);
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    this->set_parameter(rclcpp::Parameter("target_x", x));
    this->set_parameter(rclcpp::Parameter("target_y", y));
    RCLCPP_INFO(this->get_logger(), "Goal pose received: x=%.3f, y=%.3f", x, y);
  }

  void onRxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // 严格校验：长度与头尾
    constexpr size_t  kRxPacketSize  = sizeof(navCommand_t);
    constexpr uint8_t kHeader        = 0x72;
    constexpr uint8_t kTailExpected  = 0x21;

    if (msg->data.size() != kRxPacketSize) {
      RCLCPP_WARN(this->get_logger(), "Invalid rx size: %zu != %zu", msg->data.size(),
                  kRxPacketSize);
      return;
    }

    if (msg->data[0] != kHeader) {
      RCLCPP_WARN(this->get_logger(), "Invalid frame header: expected 0x%02X, got 0x%02X",
                  kHeader, msg->data[0]);
      return;
    }

    // 2. 数据转换：使用 std::memcpy 将字节流复制到结构体变量中
    navCommand_t received_cmd;
    std::memcpy(&received_cmd,       // 目标地址：结构体变量的内存地址
                msg->data.data(),    // 源地址：vector数据的起始内存地址
                sizeof(navCommand_t) // 复制长度：结构体的大小
    );

    // 3. 协议验证：在转换成结构体后，检查帧尾等成员变量是否符合协议
    if (received_cmd.frame_tail != kTailExpected) {
      RCLCPP_WARN(this->get_logger(), "Invalid frame tail: expected 0x21, got 0x%02X",
                   received_cmd.frame_tail);
      return;
    }
    // 4. 调用处理函数：所有检查通过后，数据有效，进行处理
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                      "Successfully decoded navCommand_t packet.");
    last_cmd_ = received_cmd;
    // handleNavCommand(last_cmd_);
    // RCLCPP_INFO(this->get_logger(), "Time_test: %f Now: %f", received_cmd.time_test,
    //             static_cast<float>(this->now().seconds() - 1759396608.000000));
  }

  void handleNavCommand(const navCommand_t& cmd) {
    // Best-effort mirror of hp/ammo to BT node params
    static const std::string kBtNodeName = "/rm_bt_decision_node";
    if (!bt_param_client_) {
      bt_param_client_ =
          std::make_shared<rclcpp::SyncParametersClient>(this->shared_from_this(), kBtNodeName);
    }
    if (!bt_param_client_->service_is_ready()) return;

    // Parameter names (fallback to defaults if not configured)
    std::string hp_param   = "hp";
    std::string ammo_param = "ammo";

    // Write values
    std::vector<rclcpp::Parameter> ps = {
        rclcpp::Parameter(hp_param, static_cast<double>(cmd.hp_remain)),
        rclcpp::Parameter(ammo_param, static_cast<double>(cmd.bullet_remain)),
    };
    try {
      (void) bt_param_client_->set_parameters(ps);
    } catch (...) {
      // ignore
    }

    // auto cmd.sentry_command;
    // patrol_group_pub_->publish(std_msgs::msg::String());
  }

  void publishTxPacket() {
    nav_info_.frame_header = 0x72;
    nav_info_.frame_tail   = 0x4D;
    // nav_info_.x_speed = 0.2f;
    // nav_info_.y_speed = 0.0f;

    double target_x = 0.0, target_y = 0.0;
    (void) this->get_parameter("target_x", target_x);
    (void) this->get_parameter("target_y", target_y);
    nav_info_.x_target  = static_cast<float>(target_x);
    nav_info_.y_target  = static_cast<float>(target_y);
    // nav_info_.time_test = static_cast<float>(this->now().seconds() - 1759396608.000000);
    // RCLCPP_INFO(this->get_logger(), "send: %f", nav_info_.time_test);

    // 打印发送给电控的数据（tail:0x4D）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "TX -> x_speed: %.3f y_speed: %.3f x_current: %.3f y_current: %.3f x_target: %.3f y_target: %.3f",
        nav_info_.x_speed, nav_info_.y_speed, 
        nav_info_.x_current, nav_info_.y_current,
        nav_info_.x_target, nav_info_.y_target);

    std_msgs::msg::UInt8MultiArray out_msg;
    const uint8_t*                 byte_ptr  = reinterpret_cast<const uint8_t*>(&nav_info_);
    size_t                         data_size = sizeof(navInfo_t);
    out_msg.data.assign(byte_ptr, byte_ptr + data_size);
    tx_pub_->publish(out_msg);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          patrol_group_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  rclcpp::TimerBase::SharedPtr tx_timer_;

  navInfo_t    nav_info_{};
  navCommand_t last_cmd_{};
  uint8_t      last_stop_{0};
  uint8_t      last_color_{0};

  // Parameter client for BT node
  std::shared_ptr<rclcpp::SyncParametersClient> bt_param_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandlerNode>());
  rclcpp::shutdown();
  return 0;
}
