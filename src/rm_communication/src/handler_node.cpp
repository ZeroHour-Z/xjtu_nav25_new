/**
 * @file handler_node.cpp
 * @brief 通信处理节点 - 负责与电控的通信协议转换
 * 
 * 功能：
 * 1. 接收电控数据并解析
 * 2. 发送导航信息给电控
 * 3. 订阅 region_detector 发布的区域状态
 * 4. 根据电控状态设置行为树参数（chase, hp, ammo）
 * 5. 发布追击点供行为树使用
 */

#include "protocol.h"
#include "rm_communication/packet_utils.hpp"

#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class HandlerNode: public rclcpp::Node {
public:
    HandlerNode(): Node("handler_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // 声明追击相关参数
        this->declare_parameter<double>("chase_min_distance", 0.25); // 自身与敌人的最小追击距离
        this->declare_parameter<std::string>("chase_topic", "/chase_point");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<bool>("enable_chase", true); // 是否启用追击功能

        chase_min_distance_ = this->get_parameter("chase_min_distance").as_double();
        chase_topic_ = this->get_parameter("chase_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        enable_chase_ = this->get_parameter("enable_chase").as_bool();

        RCLCPP_INFO(this->get_logger(), "Chase min distance: %.2f m", chase_min_distance_);

        // 发布器
        patrol_group_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol_group", 10);
        tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rm_comm/tx_packet", 10);

        // 追击目标点发布器（供行为树使用）
        chase_point_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(chase_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Chase mode enabled: %s", enable_chase_ ? "true" : "false");

        // 电控数据订阅
        rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/rx_packet",
            100,
            std::bind(&HandlerNode::onRxPacket, this, std::placeholders::_1)
        );

        // 速度命令订阅
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&HandlerNode::onCmdVel, this, std::placeholders::_1)
        );

        // 里程计订阅
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&HandlerNode::onOdom, this, std::placeholders::_1)
        );

        // 目标点订阅
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            rclcpp::QoS(10).best_effort(),
            std::bind(&HandlerNode::onGoalPose, this, std::placeholders::_1)
        );

        // 订阅区域检测节点发布的区域类型和期望航向角
        region_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/region_type",
            10,
            std::bind(&HandlerNode::onRegionType, this, std::placeholders::_1)
        );

        region_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/region_yaw_desired",
            10,
            std::bind(&HandlerNode::onRegionYaw, this, std::placeholders::_1)
        );

        // 声明参数
        this->declare_parameter<double>("tx_hz", 100.0);
        this->declare_parameter<double>("target_x", 0.0);
        this->declare_parameter<double>("target_y", 0.0);
        this->declare_parameter<double>("yaw_desired", 0.0);

        double hz = 100.0;
        this->get_parameter("tx_hz", hz);
        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz))),
            std::bind(&HandlerNode::publishTxPacket, this)
        );

        // 定时更新 map 坐标系下的航向角（200Hz 以应对高速陀螺旋转）
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5), // 200Hz 更新 TF
            std::bind(&HandlerNode::updateMapYaw, this)
        );

        RCLCPP_INFO(this->get_logger(), "handler_node started");
    }

    ~HandlerNode() override = default;

private:
    // map 坐标系下的航向角和角速度
    double yaw_in_map_ { 0.0 };
    double prev_yaw_in_map_ { 0.0 };
    double estimated_wz_ { 0.0 };
    rclcpp::Time prev_tf_time_;
    bool has_prev_tf_ { false };

    // 区域检测状态（来自 region_detector_node）
    uint8_t current_region_ { 1 }; // 默认 flat
    float region_yaw_desired_ { 0.0f };
    bool region_active_ { false }; // 是否在特殊区域中

    // 区域类型回调
    void onRegionType(const std_msgs::msg::UInt8::SharedPtr msg) {
        current_region_ = msg->data;
        region_active_ = (current_region_ != 1); // 非 flat 即为特殊区域
    }

    // 区域期望航向角回调
    void onRegionYaw(const std_msgs::msg::Float32::SharedPtr msg) {
        region_yaw_desired_ = msg->data;
    }

    // 定时从 TF 更新 map 坐标系下的航向角
    void updateMapYaw() {
        try {
            auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            tf2::Quaternion q;
            q.setX(tf.transform.rotation.x);
            q.setY(tf.transform.rotation.y);
            q.setZ(tf.transform.rotation.z);
            q.setW(tf.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // 估计角速度用于预测
            rclcpp::Time now = this->now();
            if (has_prev_tf_) {
                double dt = (now - prev_tf_time_).seconds();
                if (dt > 1e-6) {
                    double dyaw = yaw - prev_yaw_in_map_;
                    while (dyaw > M_PI)
                        dyaw -= 2.0 * M_PI;
                    while (dyaw < -M_PI)
                        dyaw += 2.0 * M_PI;
                    estimated_wz_ = dyaw / dt;
                }
            }
            prev_yaw_in_map_ = yaw;
            prev_tf_time_ = now;
            has_prev_tf_ = true;
            yaw_in_map_ = yaw;
        } catch (const tf2::TransformException& ex) {
            // TF not ready yet
        }
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        const double vx_map = msg->linear.x;
        const double vy_map = msg->linear.y;

        // 预测未来的航向角（补偿延迟）
        const double predict_time = 0.025;
        double predicted_yaw = yaw_in_map_ + estimated_wz_ * predict_time;

        const double cos_yaw = std::cos(predicted_yaw);
        const double sin_yaw = std::sin(predicted_yaw);

        // map 坐标系 -> base_link 坐标系
        nav_info_.x_speed = static_cast<float>(cos_yaw * vx_map + sin_yaw * vy_map);
        nav_info_.y_speed = static_cast<float>(-sin_yaw * vx_map + cos_yaw * vy_map);
        nav_info_.yaw_desired = msg->angular.z;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        nav_info_.x_current = static_cast<float>(msg->pose.pose.position.x);
        nav_info_.y_current = static_cast<float>(msg->pose.pose.position.y);

        const auto& q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        nav_info_.yaw_current = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    }

    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        this->set_parameter(rclcpp::Parameter("target_x", x));
        this->set_parameter(rclcpp::Parameter("target_y", y));
        RCLCPP_INFO(this->get_logger(), "Goal pose received: x=%.3f, y=%.3f", x, y);
    }

    void onRxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        constexpr size_t kRxPacketSize = sizeof(navCommand_t);
        constexpr uint8_t kHeader = 0x72;
        constexpr uint8_t kTailExpected = 0x21;

        if (msg->data.size() != kRxPacketSize) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid rx size: %zu != %zu",
                msg->data.size(),
                kRxPacketSize
            );
            return;
        }

        if (msg->data[0] != kHeader) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid frame header: expected 0x%02X, got 0x%02X",
                kHeader,
                msg->data[0]
            );
            return;
        }

        navCommand_t received_cmd;
        std::memcpy(&received_cmd, msg->data.data(), sizeof(navCommand_t));

        if (received_cmd.frame_tail != kTailExpected) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid frame tail: expected 0x21, got 0x%02X",
                received_cmd.frame_tail
            );
            return;
        }

        last_cmd_ = received_cmd;

        // 发布追击目标点（当敌方位置有效时）
        publishChasePoint(received_cmd);

        // 根据电控状态设置行为树参数
        updateBehaviorTreeParams(received_cmd);
    }

    // 根据电控状态更新行为树参数
    void updateBehaviorTreeParams(const navCommand_t& cmd) {
        static const std::string kBtNodeName = "/rm_bt_decision_node";

        // 初始化参数客户端
        if (!bt_param_client_) {
            bt_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
                this->shared_from_this(),
                kBtNodeName
            );
        }

        if (!bt_param_client_->service_is_ready()) {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "BT node parameter service not ready"
            );
            return;
        }

        // 根据电控状态确定是否需要追击
        // attack(1) 和 pursuit(7) 状态都触发追击
        bool should_chase =
            (cmd.eSentryState == sentry_state_e::attack
             || cmd.eSentryState == sentry_state_e::pursuit);

        // 【odom坐标系】检查敌方位置有效性
        // 与 publishChasePoint 中的逻辑保持一致：检查目标与当前位置的距离
        constexpr double kStopDistanceThreshold = 0.2; // 20cm范围内视为停止指令
        double dx_odom = cmd.enemy_x - nav_info_.x_current;
        double dy_odom = cmd.enemy_y - nav_info_.y_current;
        double distance_in_odom = std::sqrt(dx_odom * dx_odom + dy_odom * dy_odom);
        bool enemy_valid = (distance_in_odom >= kStopDistanceThreshold);

        // 只有在追击状态且敌方位置有效时才真正追击
        bool chase_enabled = should_chase && enemy_valid;

        // 巡逻状态
        bool patrol_enabled = (cmd.eSentryState == sentry_state_e::patrol);

        // 设置行为树参数
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("chase", chase_enabled),
            rclcpp::Parameter("patrol", patrol_enabled),
            rclcpp::Parameter("hp", static_cast<double>(cmd.hp_remain)),
            rclcpp::Parameter("ammo", static_cast<double>(cmd.bullet_remain)),
        };

        try {
            auto results = bt_param_client_->set_parameters(params);
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Set BT params: chase=%s, patrol=%s, hp=%u, ammo=%u, state=%d",
                chase_enabled ? "true" : "false",
                patrol_enabled ? "true" : "false",
                cmd.hp_remain,
                cmd.bullet_remain,
                static_cast<int>(cmd.eSentryState)
            );
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to set BT params: %s",
                e.what()
            );
        }

        // 根据巡逻状态发布巡逻组
        if (cmd.eSentryState == sentry_state_e::patrol) {
            std_msgs::msg::String patrol_msg;
            // 可以根据其他条件选择巡逻组，这里默认 A 组
            patrol_msg.data = "A";
            patrol_group_pub_->publish(patrol_msg);

            // 【关键修复】进入 patrol 模式时，发布当前位置作为目标点，清除之前的导航任务
            try {
                auto current_tf =
                    tf_buffer_.lookupTransform(map_frame_, "base_link", tf2::TimePointZero);

                geometry_msgs::msg::PoseStamped patrol_stop_pose;
                patrol_stop_pose.header.stamp = this->now();
                patrol_stop_pose.header.frame_id = map_frame_;
                patrol_stop_pose.pose.position.x = current_tf.transform.translation.x;
                patrol_stop_pose.pose.position.y = current_tf.transform.translation.y;
                patrol_stop_pose.pose.position.z = 0.0;
                patrol_stop_pose.pose.orientation = current_tf.transform.rotation;

                chase_point_pub_->publish(patrol_stop_pose);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "Patrol mode: Published current position to clear previous nav goal: map(%.2f, %.2f)",
                    patrol_stop_pose.pose.position.x,
                    patrol_stop_pose.pose.position.y
                );
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Failed to get current position for patrol stop: %s",
                    ex.what()
                );
            }
        }

        // 记录状态变化
        if (cmd.eSentryState != last_sentry_state_) {
            const char* state_names[] = {
                "standby",      "attack", "patrol",  "stationary_defense", "constrained_defense",
                "error",        "logic",  "pursuit", "go_attack_outpost",  "hit_energy_buff",
                "occupy_point", "repel"
            };
            int state_idx = static_cast<int>(cmd.eSentryState);
            const char* state_name =
                (state_idx >= 0
                 && state_idx < static_cast<int>(sizeof(state_names) / sizeof(state_names[0])))
                ? state_names[state_idx]
                : "unknown";
            RCLCPP_INFO(this->get_logger(), "Sentry state changed: %s (%d)", state_name, state_idx);
            last_sentry_state_ = static_cast<sentry_state_e>(cmd.eSentryState);
        }
    }

    // 发布追击目标点
    void publishChasePoint(const navCommand_t& cmd) {
        // 检查追击功能是否启用
        if (!enable_chase_) {
            return;
        }

        // 【关键修改】检查敌方位置有效性：电控发送 odom 坐标系下的点
        // 当电控想要停止时，会发送车体当前位置 (x_current, y_current)
        // 因此需要检查目标点与当前位置的距离，而不是与原点的距离
        constexpr double kStopDistanceThreshold = 0.2; // 20cm范围内视为停止指令

        double dx_odom = cmd.enemy_x - nav_info_.x_current;
        double dy_odom = cmd.enemy_y - nav_info_.y_current;
        double distance_in_odom = std::sqrt(dx_odom * dx_odom + dy_odom * dy_odom);

        if (distance_in_odom < kStopDistanceThreshold) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "Stop command detected: target odom(%.2f, %.2f) ≈ current odom(%.2f, %.2f), dist=%.3fm - Publishing current position as stop target",
                cmd.enemy_x,
                cmd.enemy_y,
                nav_info_.x_current,
                nav_info_.y_current,
                distance_in_odom
            );

            // 【关键修改】主动发布当前位置作为目标点，让车立即停止
            // 获取当前车体在 map 坐标系下的位置
            try {
                auto current_tf =
                    tf_buffer_.lookupTransform(map_frame_, "base_link", tf2::TimePointZero);

                // 发布当前位置作为追击目标
                geometry_msgs::msg::PoseStamped stop_pose;
                stop_pose.header.stamp = this->now();
                stop_pose.header.frame_id = map_frame_;
                stop_pose.pose.position.x = current_tf.transform.translation.x;
                stop_pose.pose.position.y = current_tf.transform.translation.y;
                stop_pose.pose.position.z = 0.0;
                stop_pose.pose.orientation = current_tf.transform.rotation;

                chase_point_pub_->publish(stop_pose);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "Published STOP target at current position: map(%.2f, %.2f)",
                    stop_pose.pose.position.x,
                    stop_pose.pose.position.y
                );
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Failed to get current position for stop command: %s",
                    ex.what()
                );
            }

            return;
        }

        // 电控发来的敌方坐标是 odom 坐标系，需要转换到 map 坐标系
        geometry_msgs::msg::PoseStamped enemy_in_odom;
        enemy_in_odom.header.stamp = this->now();
        enemy_in_odom.header.frame_id = "odom";
        enemy_in_odom.pose.position.x = cmd.enemy_x;
        enemy_in_odom.pose.position.y = cmd.enemy_y;
        enemy_in_odom.pose.position.z = 0.0;
        enemy_in_odom.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped enemy_in_map;
        try {
            // 获取 odom -> map 的变换并转换坐标
            enemy_in_map =
                tf_buffer_.transform(enemy_in_odom, map_frame_, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Failed to transform enemy position from odom to %s: %s",
                map_frame_.c_str(),
                ex.what()
            );
            return;
        }

        double enemy_map_x = enemy_in_map.pose.position.x;
        double enemy_map_y = enemy_in_map.pose.position.y;

        // 构建 PoseStamped 消息（已经是 map 坐标系）
        geometry_msgs::msg::PoseStamped chase_pose;
        chase_pose.header.stamp = this->now();
        chase_pose.header.frame_id = map_frame_;
        chase_pose.pose.position.x = enemy_map_x;
        chase_pose.pose.position.y = enemy_map_y;
        chase_pose.pose.position.z = 0.0;
        // 默认朝向
        chase_pose.pose.orientation.x = 0.0;
        chase_pose.pose.orientation.y = 0.0;
        chase_pose.pose.orientation.z = 0.0;
        chase_pose.pose.orientation.w = 1.0;

        chase_point_pub_->publish(chase_pose);

        // 同步更新 target_x/target_y 参数，使 TX 包中的目标位置也更新
        this->set_parameter(rclcpp::Parameter("target_x", enemy_map_x));
        this->set_parameter(rclcpp::Parameter("target_y", enemy_map_y));

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "Published chase point: odom(%.2f, %.2f) -> map(%.2f, %.2f), self_odom(%.2f, %.2f), dist=%.2fm",
            cmd.enemy_x,
            cmd.enemy_y,
            enemy_map_x,
            enemy_map_y,
            nav_info_.x_current,
            nav_info_.y_current,
            distance_in_odom
        );
    }

    void publishTxPacket() {
        nav_info_.frame_header = 0x72;
        nav_info_.frame_tail = 0x4D;

        // 使用 region_detector 发布的区域类型
        nav_info_.sentry_region = current_region_;

        // 如果在特殊区域（如颠簸区域），使用区域指定的航向角
        if (region_active_ && current_region_ == sentry_region::fluctuate) {
            nav_info_.yaw_desired = region_yaw_desired_;
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "In fluctuate region, yaw_desired=%.2f",
                region_yaw_desired_
            );
        }

        double target_x = 0.0, target_y = 0.0;
        (void)this->get_parameter("target_x", target_x);
        (void)this->get_parameter("target_y", target_y);
        nav_info_.x_target = static_cast<float>(target_x);
        nav_info_.y_target = static_cast<float>(target_y);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "TX -> x_speed: %.3f y_speed: %.3f x_current: %.3f y_current: %.3f "
            "x_target: %.3f y_target: %.3f yaw_current: %.3f yaw_desired: %.3f sentry_region: %d",
            nav_info_.x_speed,
            nav_info_.y_speed,
            nav_info_.x_current,
            nav_info_.y_current,
            nav_info_.x_target,
            nav_info_.y_target,
            nav_info_.yaw_current,
            nav_info_.yaw_desired,
            nav_info_.sentry_region
        );

        std_msgs::msg::UInt8MultiArray out_msg;
        const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(&nav_info_);
        size_t data_size = sizeof(navInfo_t);
        out_msg.data.assign(byte_ptr, byte_ptr + data_size);
        tx_pub_->publish(out_msg);
    }

    // 发布器
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr patrol_group_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr chase_point_pub_;

    // 上次状态（用于检测状态变化）
    sentry_state_e last_sentry_state_ { sentry_state_e::standby };

    // 订阅器
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr region_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr region_yaw_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr tx_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // TF 监听器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 数据
    navInfo_t nav_info_ {};
    navCommand_t last_cmd_ {};

    // 追击相关参数
    double chase_min_distance_ { 0.25 }; // 自身与敌人的最小追击距离
    std::string chase_topic_ { "/chase_point" };
    std::string map_frame_ { "map" };
    bool enable_chase_ { true };

    // BT 节点参数客户端
    std::shared_ptr<rclcpp::SyncParametersClient> bt_param_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandlerNode>());
    rclcpp::shutdown();
    return 0;
}
