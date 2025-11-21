#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class PathFollowerNode : public rclcpp::Node {
public:
  explicit PathFollowerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("path_follower", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Parameters
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("planner_action_name", "compute_path_to_pose");
    declare_parameter<std::string>("target_twist_map_topic", "/target_twist_map");
    declare_parameter<double>("goal_x", 0.0);
    declare_parameter<double>("goal_y", 0.0);
    declare_parameter<double>("goal_yaw", 0.0);
    declare_parameter<std::string>("goal_frame_id", "map");
    declare_parameter<double>("desired_speed", 0.6);
    declare_parameter<double>("lookahead_distance", 0.5);
    declare_parameter<double>("replan_period", 1.0);
    declare_parameter<double>("goal_tolerance", 0.2);
    declare_parameter<double>("yaw_gain", 1.0);
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    declare_parameter<bool>("enable_lowpass_smoothing", true);
    declare_parameter<double>("smoothing_alpha", 0.8); // 0..1, 越大越平滑
    declare_parameter<bool>("enable_slew_limit", true);
    declare_parameter<double>("max_linear_accel", 1.5);  // m/s^2
    declare_parameter<double>("max_angular_accel", 3.0); // rad/s^2
    declare_parameter<bool>("replan_on_goal_update", true);
    declare_parameter<bool>("replan_on_timer", true);

    global_frame_             = get_parameter("global_frame").as_string();
    base_frame_               = get_parameter("base_frame").as_string();
    target_twist_topic_       = get_parameter("target_twist_map_topic").as_string();
    desired_speed_            = get_parameter("desired_speed").as_double();
    lookahead_distance_       = get_parameter("lookahead_distance").as_double();
    replan_period_            = get_parameter("replan_period").as_double();
    goal_tolerance_           = get_parameter("goal_tolerance").as_double();
    yaw_gain_                 = get_parameter("yaw_gain").as_double();
    enable_lowpass_smoothing_ = get_parameter("enable_lowpass_smoothing").as_bool();
    smoothing_alpha_       = std::clamp(get_parameter("smoothing_alpha").as_double(), 0.0, 0.999);
    enable_slew_limit_     = get_parameter("enable_slew_limit").as_bool();
    max_linear_accel_      = std::max(0.0, get_parameter("max_linear_accel").as_double());
    max_angular_accel_     = std::max(0.0, get_parameter("max_angular_accel").as_double());
    replan_on_goal_update_ = get_parameter("replan_on_goal_update").as_bool();
    replan_on_timer_       = get_parameter("replan_on_timer").as_bool();

    std::string planner_action_name = get_parameter("planner_action_name").as_string();
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        this, planner_action_name);

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(target_twist_topic_, 10);

    // Subscribe to goal pose from RViz
    const auto goal_topic = get_parameter("goal_pose_topic").as_string();
    goal_sub_             = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          // Update internal goal parameters so next replan uses new goal
          double          roll, pitch, yaw;
          tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                                        msg->pose.orientation.z, msg->pose.orientation.w);
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          std::vector<rclcpp::Parameter> params;
          params.emplace_back(rclcpp::Parameter("goal_frame_id", msg->header.frame_id));
          params.emplace_back(rclcpp::Parameter("goal_x", msg->pose.position.x));
          params.emplace_back(rclcpp::Parameter("goal_y", msg->pose.position.y));
          params.emplace_back(rclcpp::Parameter("goal_yaw", yaw));
          (void) this->set_parameters(params);
          RCLCPP_INFO(this->get_logger(), "Received new goal from %s: (%.3f, %.3f, %.3f)",
                                  msg->header.frame_id.c_str(), msg->pose.position.x, msg->pose.position.y,
                                  yaw);
          if (replan_on_goal_update_) {
            this->onReplanTimer();
          }
        });

    if (replan_on_timer_) {
      replan_timer_ =
          create_wall_timer(std::chrono::duration<double>(std::max(0.1, replan_period_)),
                            std::bind(&PathFollowerNode::onReplanTimer, this));
    }
    control_timer_ = create_wall_timer(20ms, std::bind(&PathFollowerNode::onControlTimer, this));

    RCLCPP_INFO(get_logger(), "PathFollowerNode started. Publishing target twists on %s",
                target_twist_topic_.c_str());
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    double          r, p, y;
    tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tfq).getRPY(r, p, y);
    return y;
  }

  void onReplanTimer() {
    if (!action_client_->wait_for_action_server(100ms)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Planner action server not available yet");
      return;
    }

    auto                            goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped goal;

    // Start pose from TF current base pose in global frame
    try {
      auto tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      start.header.stamp     = tf.header.stamp;
      start.header.frame_id  = global_frame_;
      start.pose.position.x  = tf.transform.translation.x;
      start.pose.position.y  = tf.transform.translation.y;
      start.pose.position.z  = 0.0;
      start.pose.orientation = tf.transform.rotation;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup for start failed: %s",
                           ex.what());
      return;
    }

    const auto   goal_frame_id = get_parameter("goal_frame_id").as_string();
    const double gx            = get_parameter("goal_x").as_double();
    const double gy            = get_parameter("goal_y").as_double();
    const double gyaw          = get_parameter("goal_yaw").as_double();

    goal.header.stamp    = now();
    goal.header.frame_id = goal_frame_id;
    goal.pose.position.x = gx;
    goal.pose.position.y = gy;
    goal.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, gyaw);
    goal.pose.orientation = tf2::toMsg(q);

    goal_msg.start     = start;
    goal_msg.goal      = goal;
    goal_msg.use_start = true;

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto future) {
      auto gh = future.get();
      if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "ComputePath goal rejected");
      }
    };
    send_goal_options.result_callback = [this](const auto& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_path_     = result.result->path;
        last_closest_idx_ = 0; // 重置推进索引
        RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", current_path_.poses.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "ComputePath failed (code %d)",
                    static_cast<int>(result.code));
      }
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  bool findLookaheadTarget(const geometry_msgs::msg::Pose& base_in_map,
                           geometry_msgs::msg::Point& target_pt, double& path_heading) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (current_path_.poses.empty()) {
      return false;
    }

    // Find closest point index (单调不回退)
    double min_dist    = std::numeric_limits<double>::infinity();
    size_t closest_idx = last_closest_idx_;
    for (size_t i = last_closest_idx_; i < current_path_.poses.size(); ++i) {
      double dx = current_path_.poses[i].pose.position.x - base_in_map.position.x;
      double dy = current_path_.poses[i].pose.position.y - base_in_map.position.y;
      double d  = std::hypot(dx, dy);
      if (d < min_dist) {
        min_dist    = d;
        closest_idx = i;
      }
    }
    // 防止抖动：限制回退
    if (closest_idx < last_closest_idx_) {
      closest_idx = last_closest_idx_;
    }

    // Lookahead point ahead of closest by distance
    double accum      = 0.0;
    size_t target_idx = closest_idx;
    for (size_t i = closest_idx + 1; i < current_path_.poses.size(); ++i) {
      double dx =
          current_path_.poses[i].pose.position.x - current_path_.poses[i - 1].pose.position.x;
      double dy =
          current_path_.poses[i].pose.position.y - current_path_.poses[i - 1].pose.position.y;
      accum += std::hypot(dx, dy);
      if (accum >= lookahead_distance_) {
        target_idx = i;
        break;
      }
    }

    if (target_idx >= current_path_.poses.size()) {
      target_idx = current_path_.poses.size() - 1;
    }

    target_pt = current_path_.poses[target_idx].pose.position;

    // Path heading at target segment
    if (target_idx == 0) {
      path_heading = yawFromQuaternion(current_path_.poses[0].pose.orientation);
    } else {
      double dx = current_path_.poses[target_idx].pose.position.x -
                  current_path_.poses[target_idx - 1].pose.position.x;
      double dy = current_path_.poses[target_idx].pose.position.y -
                  current_path_.poses[target_idx - 1].pose.position.y;
      path_heading = std::atan2(dy, dx);
    }
    // 更新推进索引（仅前进）
    last_closest_idx_ = std::max(last_closest_idx_, closest_idx);
    return true;
  }

  void onControlTimer() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::Pose base_in_map;
    base_in_map.position.x  = tf.transform.translation.x;
    base_in_map.position.y  = tf.transform.translation.y;
    base_in_map.position.z  = 0.0;
    base_in_map.orientation = tf.transform.rotation;

    geometry_msgs::msg::Point target;
    double                    path_heading = 0.0;
    if (!findLookaheadTarget(base_in_map, target, path_heading)) {
      return;
    }

    // Stop if at goal
    double goal_dist =
        std::hypot(target.x - base_in_map.position.x, target.y - base_in_map.position.y);
    if (goal_dist <= goal_tolerance_) {
      geometry_msgs::msg::TwistStamped zero;
      zero.header.stamp    = now();
      zero.header.frame_id = global_frame_;
      zero.twist.linear.x  = 0.0;
      zero.twist.linear.y  = 0.0;
      zero.twist.angular.z = 0.0;
      twist_pub_->publish(zero);
      return;
    }

    // Direction vector in map frame
    double dx   = target.x - base_in_map.position.x;
    double dy   = target.y - base_in_map.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
      return;
    }
    dx /= dist;
    dy /= dist;

    double vx_m = desired_speed_ * dx;
    double vy_m = desired_speed_ * dy;

    double yaw_base = yawFromQuaternion(base_in_map.orientation);
    double yaw_err =
        std::atan2(std::sin(path_heading - yaw_base), std::cos(path_heading - yaw_base));
    double wz = yaw_gain_ * yaw_err;

    // 低通滤波（平滑）
    if (enable_lowpass_smoothing_ && have_prev_cmd_) {
      double a = smoothing_alpha_;
      vx_m     = a * prev_vx_m_ + (1.0 - a) * vx_m;
      vy_m     = a * prev_vy_m_ + (1.0 - a) * vy_m;
      wz       = a * prev_wz_ + (1.0 - a) * wz;
    }

    // 加速度限幅（斜率限制）
    if (enable_slew_limit_) {
      const double dt     = 0.02; // 20ms 控制周期
      double       max_dv = max_linear_accel_ * dt;
      double       max_dw = max_angular_accel_ * dt;
      if (have_prev_cmd_) {
        vx_m = std::clamp(vx_m, prev_vx_m_ - max_dv, prev_vx_m_ + max_dv);
        vy_m = std::clamp(vy_m, prev_vy_m_ - max_dv, prev_vy_m_ + max_dv);
        wz   = std::clamp(wz, prev_wz_ - max_dw, prev_wz_ + max_dw);
      }
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = now();
    cmd.header.frame_id = global_frame_;
    cmd.twist.linear.x  = vx_m;
    cmd.twist.linear.y  = vy_m;
    cmd.twist.angular.z = wz;
    twist_pub_->publish(cmd);

    prev_vx_m_     = vx_m;
    prev_vy_m_     = vy_m;
    prev_wz_       = wz;
    have_prev_cmd_ = true;
  }

  std::string global_frame_;
  std::string base_frame_;
  std::string target_twist_topic_;
  double      desired_speed_{0.6};
  double      lookahead_distance_{0.5};
  double      replan_period_{1.0};
  double      goal_tolerance_{0.2};
  double      yaw_gain_{1.0};
  bool        enable_lowpass_smoothing_{true};
  double      smoothing_alpha_{0.8};
  bool        enable_slew_limit_{true};
  double      max_linear_accel_{1.5};
  double      max_angular_accel_{3.0};
  bool        replan_on_goal_update_{true};
  bool        replan_on_timer_{true};

  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr         twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr       goal_sub_;
  rclcpp::TimerBase::SharedPtr                                           replan_timer_;
  rclcpp::TimerBase::SharedPtr                                           control_timer_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Path current_path_;
  std::mutex          path_mutex_;
  size_t              last_closest_idx_{0};
  bool                have_prev_cmd_{false};
  double              prev_vx_m_{0.0};
  double              prev_vy_m_{0.0};
  double              prev_wz_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}