#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "global_velocity_controller/simulator_2d.hpp"
#include "global_velocity_controller/types.hpp"
#include "global_velocity_controller/velocity_planner.hpp"

using std::placeholders::_1;

class GlobalVelocityControllerNode : public rclcpp::Node {
   public:
    GlobalVelocityControllerNode()
        : Node("global_velocity_controller"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
        // Parameters
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        declare_parameter<std::string>("odom_topic", "/odom");
        declare_parameter<std::string>("target_twist_map_topic", "/target_twist_map");
        declare_parameter<std::string>("path_topic", "/plan");
        declare_parameter<std::string>("global_costmap_topic", "/global_costmap/costmap");

        declare_parameter<double>("kp_xy", 1.0);
        declare_parameter<double>("ki_xy", 0.0);
        declare_parameter<double>("kd_xy", 0.0);

        declare_parameter<double>("kp_yaw", 1.0);
        declare_parameter<double>("ki_yaw", 0.0);
        declare_parameter<double>("kd_yaw", 0.0);

        declare_parameter<double>("max_vx", 1.0);
        declare_parameter<double>("max_vy", 1.0);
        declare_parameter<double>("max_wz", 1.0);

        declare_parameter<double>("anti_windup_integral_limit_xy", 1.0);
        declare_parameter<double>("anti_windup_integral_limit_yaw", 1.0);

        declare_parameter<double>("lookahead_distance", 0.5);
        declare_parameter<double>("target_speed", 0.6);
        declare_parameter<double>("heading_kp", 1.0);
        declare_parameter<double>("external_override_timeout", 0.2);
        declare_parameter<double>("velocity_deadband", 0.02);
        declare_parameter<double>("yaw_deadband", 0.02);
        declare_parameter<double>("stop_speed_threshold", 0.04);
        declare_parameter<double>("stop_angular_threshold", 0.03);
        declare_parameter<double>("cmd_accel_limit_linear", 2.0);
        declare_parameter<double>("cmd_accel_limit_angular", 4.0);
        declare_parameter<double>("slow_down_distance", 0.6);
        declare_parameter<double>("min_target_speed", 0.03);
        declare_parameter<double>("goal_position_tolerance", 0.06);
        declare_parameter<double>("goal_yaw_tolerance", 0.06);
        declare_parameter<double>("final_heading_switch_distance", 0.3);
        declare_parameter<bool>("enable_dynamic_lookahead", true);
        declare_parameter<double>("min_lookahead_distance", 0.3);
        declare_parameter<double>("max_lookahead_distance", 1.0);
        declare_parameter<double>("curvature_window_distance", 1.0);
        declare_parameter<double>("curvature_low", 0.05);
        declare_parameter<double>("curvature_high", 0.8);

        declare_parameter<bool>("simulate", false);
        declare_parameter<double>("sim_init_x", 0.0);
        declare_parameter<double>("sim_init_y", 0.0);
        declare_parameter<double>("sim_init_yaw", 0.0);
        declare_parameter<double>("sim_linear_noise_std", 0.05);
        declare_parameter<double>("sim_angular_noise_std", 0.02);
        declare_parameter<int>("sim_noise_seed", 42);
        declare_parameter<double>("sim_process_linear_noise_std", 0.0);
        declare_parameter<double>("sim_process_angular_noise_std", 0.0);

        // New: dt cap for control and sim substep size
        declare_parameter<double>("max_dt", 0.05);
        declare_parameter<double>("sim_integration_step_dt", 0.02);

        // Escape mode params
        declare_parameter<int>("escape_free_cost_value", 0);
        declare_parameter<int>("escape_target_cost_threshold", 0);
        declare_parameter<int>("escape_enter_cost_threshold", 1);
        declare_parameter<int>("escape_lethal_threshold", 100);
        declare_parameter<bool>("escape_treat_unknown_as_lethal", true);
        declare_parameter<double>("escape_speed", 0.4);
        declare_parameter<double>("escape_goal_tolerance", 0.05);
        declare_parameter<int>("escape_max_radius_cells", 200);

        // Get parameters
        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();

        std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
        std::string odom_topic = get_parameter("odom_topic").as_string();
        std::string target_twist_map_topic = get_parameter("target_twist_map_topic").as_string();
        std::string path_topic = get_parameter("path_topic").as_string();
        std::string costmap_topic = get_parameter("global_costmap_topic").as_string();

        // PID gains
        kp_xy_ = get_parameter("kp_xy").as_double();
        ki_xy_ = get_parameter("ki_xy").as_double();
        kd_xy_ = get_parameter("kd_xy").as_double();

        kp_yaw_ = get_parameter("kp_yaw").as_double();
        ki_yaw_ = get_parameter("ki_yaw").as_double();
        kd_yaw_ = get_parameter("kd_yaw").as_double();

        max_vx_ = get_parameter("max_vx").as_double();
        max_vy_ = get_parameter("max_vy").as_double();
        max_wz_ = get_parameter("max_wz").as_double();

        int_limit_xy_ = get_parameter("anti_windup_integral_limit_xy").as_double();
        int_limit_yaw_ = get_parameter("anti_windup_integral_limit_yaw").as_double();

        // Planner config
        planner_config_.lookahead_distance = get_parameter("lookahead_distance").as_double();
        planner_config_.target_speed = get_parameter("target_speed").as_double();
        planner_config_.heading_kp = get_parameter("heading_kp").as_double();
        external_override_timeout_ = get_parameter("external_override_timeout").as_double();

        simulate_ = get_parameter("simulate").as_bool();
        simulator_config_.init_x = get_parameter("sim_init_x").as_double();
        simulator_config_.init_y = get_parameter("sim_init_y").as_double();
        simulator_config_.init_yaw = get_parameter("sim_init_yaw").as_double();
        simulator_config_.linear_meas_noise_std = get_parameter("sim_linear_noise_std").as_double();
        simulator_config_.angular_meas_noise_std = get_parameter("sim_angular_noise_std").as_double();
        simulator_config_.noise_seed = get_parameter("sim_noise_seed").as_int();
        simulator_config_.process_linear_noise_std = get_parameter("sim_process_linear_noise_std").as_double();
        simulator_config_.process_angular_noise_std = get_parameter("sim_process_angular_noise_std").as_double();

        if (simulate_) {
            RCLCPP_INFO(get_logger(),
                        "Sim mode ON. Meas noise lin=%.3f m/s, ang=%.3f rad/s; Proc noise lin=%.3f m/s, ang=%.3f rad/s "
                        "(seed=%d)",
                        simulator_config_.linear_meas_noise_std, simulator_config_.angular_meas_noise_std,
                        simulator_config_.process_linear_noise_std, simulator_config_.process_angular_noise_std,
                        simulator_config_.noise_seed);
        }

        velocity_deadband_ = get_parameter("velocity_deadband").as_double();
        yaw_deadband_ = get_parameter("yaw_deadband").as_double();
        stop_speed_threshold_ = get_parameter("stop_speed_threshold").as_double();
        stop_angular_threshold_ = get_parameter("stop_angular_threshold").as_double();
        cmd_accel_limit_linear_ = get_parameter("cmd_accel_limit_linear").as_double();
        cmd_accel_limit_angular_ = get_parameter("cmd_accel_limit_angular").as_double();
        planner_config_.slow_down_distance = get_parameter("slow_down_distance").as_double();
        planner_config_.min_target_speed = get_parameter("min_target_speed").as_double();
        planner_config_.goal_position_tolerance = get_parameter("goal_position_tolerance").as_double();
        planner_config_.goal_yaw_tolerance = get_parameter("goal_yaw_tolerance").as_double();
        planner_config_.final_heading_switch_distance = get_parameter("final_heading_switch_distance").as_double();
        planner_config_.enable_dynamic_lookahead = get_parameter("enable_dynamic_lookahead").as_bool();
        planner_config_.min_lookahead_distance = get_parameter("min_lookahead_distance").as_double();
        planner_config_.max_lookahead_distance = get_parameter("max_lookahead_distance").as_double();
        planner_config_.curvature_window_distance = get_parameter("curvature_window_distance").as_double();
        planner_config_.curvature_low = get_parameter("curvature_low").as_double();
        planner_config_.curvature_high = get_parameter("curvature_high").as_double();

        // New: dt cap and sim step
        max_dt_ = get_parameter("max_dt").as_double();
        simulator_config_.integration_step_dt = get_parameter("sim_integration_step_dt").as_double();

        // Escape params
        escape_free_cost_value_ = get_parameter("escape_free_cost_value").as_int();
        escape_target_cost_threshold_ = get_parameter("escape_target_cost_threshold").as_int();
        escape_enter_cost_threshold_ = get_parameter("escape_enter_cost_threshold").as_int();
        escape_lethal_threshold_ = get_parameter("escape_lethal_threshold").as_int();
        escape_treat_unknown_as_lethal_ = get_parameter("escape_treat_unknown_as_lethal").as_bool();
        escape_speed_ = get_parameter("escape_speed").as_double();
        escape_goal_tolerance_ = get_parameter("escape_goal_tolerance").as_double();
        escape_max_radius_cells_ = get_parameter("escape_max_radius_cells").as_int();

        // Initialize modules
        planner_.setConfig(planner_config_);
        simulator_ = gvc::Simulator2D(simulator_config_);

        // Interfaces
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        target_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            target_twist_map_topic, 10, std::bind(&GlobalVelocityControllerNode::onTargetTwist, this, _1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 50, std::bind(&GlobalVelocityControllerNode::onOdom, this, _1));

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic, 10, std::bind(&GlobalVelocityControllerNode::onPath, this, _1));

        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic, 1, std::bind(&GlobalVelocityControllerNode::onCostmap, this, _1));

        // Timer for control loop
        control_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                           std::bind(&GlobalVelocityControllerNode::onControlTimer, this));

        RCLCPP_INFO(
            get_logger(),
            "GlobalVelocityControllerNode started. Expecting target Twist in map frame on %s, and/or Path on %s",
            target_twist_map_topic.c_str(), path_topic.c_str());
    }

   private:
    void onTargetTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_target_twist_map_ = *msg;
        has_target_ = true;
    }

    void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
        last_path_ = *msg;
        has_path_ = !last_path_.poses.empty();
        // Convert to internal path for planner
        gvc::Path2D p2d;
        p2d.poses.reserve(last_path_.poses.size());
        for (const auto &ps : last_path_.poses) {
            gvc::Pose2D pp;
            pp.x = ps.pose.position.x;
            pp.y = ps.pose.position.y;
            pp.yaw = 0.0;  // yaw along path not used except goal yaw below
            p2d.poses.push_back(pp);
        }
        // overwrite last pose yaw with provided orientation
        if (!p2d.poses.empty()) {
            const auto &q = last_path_.poses.back().pose.orientation;
            tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
            double r, p, y;
            tf2::Matrix3x3(tfq).getRPY(r, p, y);
            p2d.poses.back().yaw = y;
        }
        planner_.setPath(p2d);
        planner_.resetTracking();
    }

    void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_costmap_ = *msg;
        has_costmap_ = true;
    }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
        tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
        double r, p, y;
        tf2::Matrix3x3(tfq).getRPY(r, p, y);
        return y;
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        has_odom_ = true;
    }

    // Helpers for escape mode
    bool worldToGrid(double x, double y, int &gx, int &gy) const {
        if (!has_costmap_) return false;
        const auto &info = last_costmap_.info;
        const double rel_x = x - info.origin.position.x;
        const double rel_y = y - info.origin.position.y;
        if (info.resolution <= 0.0) return false;
        gx = static_cast<int>(std::floor(rel_x / info.resolution));
        gy = static_cast<int>(std::floor(rel_y / info.resolution));
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width) || gy >= static_cast<int>(info.height)) return false;
        return true;
    }

    bool gridToWorld(int gx, int gy, double &x, double &y) const {
        if (!has_costmap_) return false;
        const auto &info = last_costmap_.info;
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width) || gy >= static_cast<int>(info.height)) return false;
        x = info.origin.position.x + (static_cast<double>(gx) + 0.5) * info.resolution;
        y = info.origin.position.y + (static_cast<double>(gy) + 0.5) * info.resolution;
        return true;
    }

    inline bool isCostLethal(int8_t v) const {
        if (v < 0) return escape_treat_unknown_as_lethal_;
        return static_cast<int>(v) >= escape_lethal_threshold_;
    }

    bool isPoseLethal(double x, double y) const {
        int gx, gy;
        if (!worldToGrid(x, y, gx, gy)) return false;
        const size_t idx = static_cast<size_t>(gy) * last_costmap_.info.width + static_cast<size_t>(gx);
        if (idx >= last_costmap_.data.size()) return false;
        return isCostLethal(last_costmap_.data[idx]);
    }

    bool getCostAt(double x, double y, int8_t &out_cost) const {
        int gx, gy;
        if (!worldToGrid(x, y, gx, gy)) return false;
        const size_t idx = static_cast<size_t>(gy) * last_costmap_.info.width + static_cast<size_t>(gx);
        if (idx >= last_costmap_.data.size()) return false;
        out_cost = last_costmap_.data[idx];
        return true;
    }

    bool findNearestFreeCell(int start_gx, int start_gy, int &out_gx, int &out_gy) const {
        if (!has_costmap_) return false;
        const auto &info = last_costmap_.info;
        const int width = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);
        const int total = width * height;
        if (width <= 0 || height <= 0) return false;

        std::vector<uint8_t> visited(total, 0);
        std::queue<std::pair<int, int>> q;
        std::queue<int> depth_q;

        auto idx_of = [width](int x, int y) { return y * width + x; };

        auto can_step = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height) return false;
            const int idx = idx_of(x, y);
            const int8_t v = last_costmap_.data[idx];
            if (v < 0) return !escape_treat_unknown_as_lethal_;
            return static_cast<int>(v) < escape_lethal_threshold_;
        };

        auto push_if_valid = [&](int x, int y, int d) {
            if (x < 0 || y < 0 || x >= width || y >= height) return;
            const int idx = idx_of(x, y);
            if (visited[idx]) return;
            visited[idx] = 1;
            q.emplace(x, y);
            depth_q.emplace(d);
        };

        // seed with start even if it is lethal; neighbors will be filtered by can_step
        push_if_valid(start_gx, start_gy, 0);

        const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();
            int cur_d = depth_q.front();
            depth_q.pop();

            const int cidx = idx_of(cx, cy);
            if (cidx < 0 || cidx >= total) continue;

            const int8_t val = last_costmap_.data[cidx];
            if (val >= 0 && static_cast<int>(val) <= escape_target_cost_threshold_) {
                out_gx = cx;
                out_gy = cy;
                return true;
            }

            if (cur_d >= escape_max_radius_cells_) continue;

            for (int k = 0; k < 8; ++k) {
                const int nx = cx + dx8[k];
                const int ny = cy + dy8[k];
                if (can_step(nx, ny)) {
                    push_if_valid(nx, ny, cur_d + 1);
                }
            }
        }
        return false;
    }

    void onControlTimer() {
        const rclcpp::Time now = get_clock()->now();

        // Current pose and yaw
        double current_x_m = 0.0;
        double current_y_m = 0.0;
        double yaw_map_to_base = 0.0;

        if (!simulate_) {
            geometry_msgs::msg::TransformStamped tf_map_to_base;
            try {
                tf_map_to_base = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
                return;
            }
            current_x_m = tf_map_to_base.transform.translation.x;
            current_y_m = tf_map_to_base.transform.translation.y;
            yaw_map_to_base = yawFromQuaternion(tf_map_to_base.transform.rotation);
        } else {
            const auto sim_pose = simulator_.getPose();
            current_x_m = sim_pose.x;
            current_y_m = sim_pose.y;
            yaw_map_to_base = sim_pose.yaw;

            // Ensure TF is available even before any command is generated
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = map_frame_;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform.translation.x = sim_pose.x;
            tf_msg.transform.translation.y = sim_pose.y;
            tf_msg.transform.translation.z = 0.0;
            tf2::Quaternion q0;
            q0.setRPY(0.0, 0.0, sim_pose.yaw);
            tf_msg.transform.rotation = tf2::toMsg(q0);
            tf_broadcaster_->sendTransform(tf_msg);
        }

        // Current body-frame velocities
        double vx_b = 0.0;
        double vy_b = 0.0;
        double wz = 0.0;

        if (!simulate_) {
            if (!has_odom_) {
                return;
            }
            vx_b = last_odom_.twist.twist.linear.x;
            vy_b = last_odom_.twist.twist.linear.y;
            wz = last_odom_.twist.twist.angular.z;
        } else {
            gvc::Twist2D meas = simulator_.sampleMeasuredTwistBody({last_cmd_vx_, last_cmd_vy_, last_cmd_wz_});
            vx_b = meas.vx;
            vy_b = meas.vy;
            wz = meas.wz;
        }

        const double cos_yaw = std::cos(yaw_map_to_base);
        const double sin_yaw = std::sin(yaw_map_to_base);

        const double vx_m = cos_yaw * vx_b - sin_yaw * vy_b;
        const double vy_m = sin_yaw * vx_b + cos_yaw * vy_b;

        // Escape mode detection and handling
        int8_t current_cost = -1;
        const bool have_current_cost = has_costmap_ && getCostAt(current_x_m, current_y_m, current_cost);
        const bool at_target_cost =
            have_current_cost && current_cost >= 0 && static_cast<int>(current_cost) <= escape_target_cost_threshold_;

        bool enter_blocked = false;
        if (have_current_cost) {
            if (current_cost < 0) enter_blocked = escape_treat_unknown_as_lethal_;
            else
                enter_blocked = static_cast<int>(current_cost) >= escape_enter_cost_threshold_;
        }

        if (!in_escape_mode_ && enter_blocked) {
            int cgx = 0, cgy = 0;
            if (worldToGrid(current_x_m, current_y_m, cgx, cgy)) {
                int tgx = 0, tgy = 0;
                if (findNearestFreeCell(cgx, cgy, tgx, tgy)) {
                    double tx = 0.0, ty = 0.0;
                    if (gridToWorld(tgx, tgy, tx, ty)) {
                        escape_target_x_m_ = tx;
                        escape_target_y_m_ = ty;
                        in_escape_mode_ = true;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                             "Entering ESCAPE mode. Target: (%.3f, %.3f) from cost %d", tx, ty,
                                             static_cast<int>(current_cost));
                    }
                }
            }
        }

        if (in_escape_mode_) {
            if (at_target_cost) {
                in_escape_mode_ = false;
                integral_x_ = integral_y_ = integral_w_ = 0.0;
                has_prev_time_ = false;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Exited ESCAPE mode (reached cost <= %d).",
                                     escape_target_cost_threshold_);
            } else {
                double dx = escape_target_x_m_ - current_x_m;
                double dy = escape_target_y_m_ - current_y_m;
                double dist = std::hypot(dx, dy);
                if (dist < escape_goal_tolerance_) {
                    int cgx = 0, cgy = 0;
                    if (worldToGrid(current_x_m, current_y_m, cgx, cgy)) {
                        int tgx = 0, tgy = 0;
                        if (findNearestFreeCell(cgx, cgy, tgx, tgy) &&
                            gridToWorld(tgx, tgy, escape_target_x_m_, escape_target_y_m_)) {
                            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "ESCAPE re-target to (%.3f, %.3f)",
                                                 escape_target_x_m_, escape_target_y_m_);
                            dx = escape_target_x_m_ - current_x_m;
                            dy = escape_target_y_m_ - current_y_m;
                            dist = std::hypot(dx, dy);
                        } else {
                            // Cannot find free cell, stop commands and keep trying next tick
                            geometry_msgs::msg::Twist zero;
                            cmd_pub_->publish(zero);
                            last_cmd_vx_ = last_cmd_vy_ = last_cmd_wz_ = 0.0;
                            return;
                        }
                    }
                }

                if (!has_prev_time_) {
                    prev_time_ = now;
                    has_prev_time_ = true;
                    return;
                }
                const double dt_raw = (now - prev_time_).seconds();
                prev_time_ = now;
                if (dt_raw <= 0.0) {
                    return;
                }
                const double dt = std::min(dt_raw, max_dt_);

                double vx_map_cmd = 0.0;
                double vy_map_cmd = 0.0;
                if (dist > 1e-6) {
                    vx_map_cmd = escape_speed_ * dx / dist;
                    vy_map_cmd = escape_speed_ * dy / dist;
                }

                const double ux_b = cos_yaw * vx_map_cmd + sin_yaw * vy_map_cmd;
                const double uy_b = -sin_yaw * vx_map_cmd + cos_yaw * vy_map_cmd;

                const double cmd_vx_raw = std::clamp(ux_b, -max_vx_, max_vx_);
                const double cmd_vy_raw = std::clamp(uy_b, -max_vy_, max_vy_);
                const double cmd_wz_raw = 0.0;  // no rotation for escape

                const double max_dv = cmd_accel_limit_linear_ * dt;
                const double max_dw = cmd_accel_limit_angular_ * dt;

                const double dvx = std::clamp(cmd_vx_raw - last_cmd_vx_, -max_dv, max_dv);
                const double dvy = std::clamp(cmd_vy_raw - last_cmd_vy_, -max_dv, max_dv);
                const double dwz = std::clamp(cmd_wz_raw - last_cmd_wz_, -max_dw, max_dw);

                double cmd_vx_out = last_cmd_vx_ + dvx;
                double cmd_vy_out = last_cmd_vy_ + dvy;
                double cmd_wz_out = last_cmd_wz_ + dwz;

                if (std::fabs(cmd_vx_out) < velocity_deadband_) cmd_vx_out = 0.0;
                if (std::fabs(cmd_vy_out) < velocity_deadband_) cmd_vy_out = 0.0;
                if (std::fabs(cmd_wz_out) < yaw_deadband_) cmd_wz_out = 0.0;

                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = cmd_vx_out;
                cmd.linear.y = cmd_vy_out;
                cmd.angular.z = cmd_wz_out;
                cmd_pub_->publish(cmd);

                last_cmd_vx_ = cmd.linear.x;
                last_cmd_vy_ = cmd.linear.y;
                last_cmd_wz_ = cmd.angular.z;

                if (simulate_) {
                    simulator_.integrateBodyCommand({cmd_vx_out, cmd_vy_out, cmd_wz_out}, dt);

                    const auto s = simulator_.getPose();
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header.stamp = now;
                    tf_msg.header.frame_id = map_frame_;
                    tf_msg.child_frame_id = base_frame_;
                    tf_msg.transform.translation.x = s.x;
                    tf_msg.transform.translation.y = s.y;
                    tf_msg.transform.translation.z = 0.0;
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, s.yaw);
                    tf_msg.transform.rotation = tf2::toMsg(q);
                    tf_broadcaster_->sendTransform(tf_msg);
                }
                return;
            }
        }

        // Determine target in map frame: prefer recent external override; otherwise compute from path
        double target_vx_m = 0.0;
        double target_vy_m = 0.0;
        double target_wz = 0.0;

        bool have_external = false;
        if (has_target_) {
            const rclcpp::Time msg_time(last_target_twist_map_.header.stamp);
            const double age = (now - msg_time).seconds();
            if (external_override_timeout_ <= 0.0 || (age >= 0.0 && age <= external_override_timeout_)) {
                have_external = true;
            }
        }

        bool have_internal = false;
        if (!have_external) {
            have_internal =
                planner_.computeTarget(current_x_m, current_y_m, yaw_map_to_base, target_vx_m, target_vy_m, target_wz);
        }

        if (have_external) {
            target_vx_m = last_target_twist_map_.twist.linear.x;
            target_vy_m = last_target_twist_map_.twist.linear.y;
            target_wz = last_target_twist_map_.twist.angular.z;
        } else if (!have_internal) {
            geometry_msgs::msg::Twist zero;
            cmd_pub_->publish(zero);
            last_cmd_vx_ = 0.0;
            last_cmd_vy_ = 0.0;
            last_cmd_wz_ = 0.0;
            integral_x_ = 0.0;
            integral_y_ = 0.0;
            integral_w_ = 0.0;
            has_prev_time_ = false;
            return;
        }

        if (!has_prev_time_) {
            prev_time_ = now;
            has_prev_time_ = true;
            return;
        }

        const double dt_raw = (now - prev_time_).seconds();
        prev_time_ = now;
        if (dt_raw <= 0.0) {
            return;
        }
        // Cap dt to improve robustness
        const double dt = std::min(dt_raw, max_dt_);

        const double ex = target_vx_m - vx_m;
        const double ey = target_vy_m - vy_m;
        const double ew = target_wz - wz;

        integral_x_ += ex * dt;
        integral_y_ += ey * dt;
        integral_w_ += ew * dt;
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //                      "e=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f), dt=%.3f",
        //                      ex, ey, ew, target_vx_m, target_vy_m, target_wz, vx_m, vy_m, wz, dt);

        integral_x_ = std::clamp(integral_x_, -int_limit_xy_, int_limit_xy_);
        integral_y_ = std::clamp(integral_y_, -int_limit_xy_, int_limit_xy_);
        integral_w_ = std::clamp(integral_w_, -int_limit_yaw_, int_limit_yaw_);

        const double dx_err = (ex - prev_ex_) / dt;
        const double dy_err = (ey - prev_ey_) / dt;
        const double dw_err = (ew - prev_ew_) / dt;

        prev_ex_ = ex;
        prev_ey_ = ey;
        prev_ew_ = ew;

        double ux_m = kp_xy_ * ex + 0 * ki_xy_ * integral_x_ + kd_xy_ * dx_err;
        double uy_m = kp_xy_ * ey + 0 * ki_xy_ * integral_y_ + kd_xy_ * dy_err;
        // double uw = kp_yaw_ * ew + ki_yaw_ * integral_w_ + kd_yaw_ * dw_err;

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "kd=(%.3f %.3f)", kd_xy_ * dx_err,
        //                      kd_xy_ * dy_err);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "err=(%.3f %.3f)", ex, ey);

        const double ux_b = cos_yaw * ux_m + sin_yaw * uy_m;
        const double uy_b = -sin_yaw * ux_m + cos_yaw * uy_m;
        // const double ux_b = ux_m;
        // const double uy_b = uy_m;

        const double cmd_vx_raw = std::clamp(ux_b, -max_vx_, max_vx_);
        const double cmd_vy_raw = std::clamp(uy_b, -max_vy_, max_vy_);
        // const double cmd_wz_raw = std::clamp(uw, -max_wz_, max_wz_);

        const double target_speed_mag = std::hypot(target_vx_m, target_vy_m);

        double cmd_vx_out = cmd_vx_raw;
        double cmd_vy_out = cmd_vy_raw;
        double cmd_wz_out = 0;

        if (target_speed_mag < stop_speed_threshold_ && std::fabs(target_wz) < stop_angular_threshold_) {
            cmd_vx_out = 0.0;
            cmd_vy_out = 0.0;
            cmd_wz_out = 0.0;
            integral_x_ = 0.0;
            integral_y_ = 0.0;
            integral_w_ = 0.0;
        } else {
            const double max_dv = cmd_accel_limit_linear_ * dt;
            const double max_dw = cmd_accel_limit_angular_ * dt;

            const double dvx = std::clamp(cmd_vx_raw - last_cmd_vx_, -max_dv, max_dv);
            const double dvy = std::clamp(cmd_vy_raw - last_cmd_vy_, -max_dv, max_dv);
            // const double dwz = std::clamp(cmd_wz_raw - last_cmd_wz_, -max_dw, max_dw);

            cmd_vx_out = last_cmd_vx_ + dvx;
            cmd_vy_out = last_cmd_vy_ + dvy;
            // cmd_wz_out = last_cmd_wz_ + dwz;

            if (std::fabs(cmd_vx_out) < velocity_deadband_) cmd_vx_out = 0.0;
            if (std::fabs(cmd_vy_out) < velocity_deadband_) cmd_vy_out = 0.0;
            if (std::fabs(cmd_wz_out) < yaw_deadband_) cmd_wz_out = 0.0;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = cmd_vx_out;
        cmd.linear.y = cmd_vy_out;
        cmd.angular.x = yaw_map_to_base;
        cmd_pub_->publish(cmd);

        last_cmd_vx_ = cmd.linear.x;
        last_cmd_vy_ = cmd.linear.y;
        // last_cmd_wz_ = cmd.angular.z;

        // Loopback simulation: integrate pose and publish TF
        if (simulate_) {
            simulator_.integrateBodyCommand({cmd_vx_out, cmd_vy_out, cmd_wz_out}, dt);

            const auto s = simulator_.getPose();
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = map_frame_;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform.translation.x = s.x;
            tf_msg.transform.translation.y = s.y;
            tf_msg.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, s.yaw);
            tf_msg.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    std::string map_frame_;
    std::string base_frame_;

    double kp_xy_{1.0}, ki_xy_{0.0}, kd_xy_{0.0};
    double kp_yaw_{1.0}, ki_yaw_{0.0}, kd_yaw_{0.0};

    double max_vx_{1.0}, max_vy_{1.0}, max_wz_{1.0};
    double int_limit_xy_{1.0}, int_limit_yaw_{1.0};

    // Planner and simulator configs
    gvc::VelocityPlannerConfig planner_config_{};
    gvc::SimulatorConfig simulator_config_{};

    double external_override_timeout_{0.2};
    double velocity_deadband_{0.02}, yaw_deadband_{0.02};
    double stop_speed_threshold_{0.04}, stop_angular_threshold_{0.03};
    double cmd_accel_limit_linear_{2.0}, cmd_accel_limit_angular_{4.0};

    bool simulate_{false};

    // Controllers' runtime states
    double last_cmd_vx_{0.0}, last_cmd_vy_{0.0}, last_cmd_wz_{0.0};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::TwistStamped last_target_twist_map_;
    nav_msgs::msg::Odometry last_odom_;
    nav_msgs::msg::Path last_path_;
    nav_msgs::msg::OccupancyGrid last_costmap_;
    bool has_target_{false};
    bool has_odom_{false};
    bool has_path_{false};
    bool has_costmap_{false};

    bool has_prev_time_{false};
    rclcpp::Time prev_time_;

    double integral_x_{0.0}, integral_y_{0.0}, integral_w_{0.0};
    double prev_ex_{0.0}, prev_ey_{0.0}, prev_ew_{0.0};

    // New: dt cap
    double max_dt_{0.05};

    // Escape mode states
    bool in_escape_mode_{false};
    int escape_free_cost_value_{0};
    int escape_target_cost_threshold_{0};
    int escape_enter_cost_threshold_{1};
    int escape_lethal_threshold_{100};
    bool escape_treat_unknown_as_lethal_{true};
    double escape_speed_{0.4};
    double escape_goal_tolerance_{0.05};
    int escape_max_radius_cells_{200};
    double escape_target_x_m_{0.0};
    double escape_target_y_m_{0.0};

    // Modules
    gvc::VelocityPlanner planner_{};
    gvc::Simulator2D simulator_{};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalVelocityControllerNode>());
    rclcpp::shutdown();
    return 0;
}