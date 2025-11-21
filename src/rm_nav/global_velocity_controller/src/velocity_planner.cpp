#include "global_velocity_controller/velocity_planner.hpp"

#include <cmath>
#include <limits>

namespace gvc {

  VelocityPlanner::VelocityPlanner(const VelocityPlannerConfig& config) : config_(config) {}

  void VelocityPlanner::setConfig(const VelocityPlannerConfig& config) { config_ = config; }

  void VelocityPlanner::setPath(const Path2D& path) {
    path_ = path;
    resetTracking();
  }

  void VelocityPlanner::resetTracking() {
    has_last_closest_index_ = false;
    last_closest_index_     = 0;
  }

  double VelocityPlanner::computeDynamicLookahead(std::size_t start_index) const {
    if (!config_.enable_dynamic_lookahead) {
      return config_.lookahead_distance;
    }
    if (path_.poses.size() < 3) {
      return config_.lookahead_distance;
    }

    double accumulated_length_m = 0.0;
    double sum_abs_curvature    = 0.0;
    int    curvature_samples    = 0;

    std::size_t i = start_index;
    while (i + 2 < path_.poses.size() && accumulated_length_m < config_.curvature_window_distance) {
      const auto& p0 = path_.poses[i];
      const auto& p1 = path_.poses[i + 1];
      const auto& p2 = path_.poses[i + 2];

      const double dx01  = p1.x - p0.x;
      const double dy01  = p1.y - p0.y;
      const double dx12  = p2.x - p1.x;
      const double dy12  = p2.y - p1.y;
      const double len01 = std::hypot(dx01, dy01);
      const double len12 = std::hypot(dx12, dy12);

      if (len01 > 1e-6 && len12 > 1e-6) {
        const double th0     = std::atan2(dy01, dx01);
        const double th1     = std::atan2(dy12, dx12);
        double       dth     = normalizeAngle(th1 - th0);
        const double seg_len = len12;
        const double kappa   = std::fabs(dth) / std::max(seg_len, 1e-6);
        sum_abs_curvature += kappa;
        curvature_samples += 1;
      }
      accumulated_length_m += len12;
      i += 1;
    }

    const double mean_abs_curvature =
        (curvature_samples > 0) ? (sum_abs_curvature / static_cast<double>(curvature_samples))
                                : 0.0;

    double alpha = 0.0;
    if (mean_abs_curvature <= config_.curvature_low) {
      alpha = 0.0;
    } else if (mean_abs_curvature >= config_.curvature_high) {
      alpha = 1.0;
    } else {
      alpha = (mean_abs_curvature - config_.curvature_low) /
              std::max(1e-9, (config_.curvature_high - config_.curvature_low));
    }

    const double lookahead =
        config_.max_lookahead_distance -
        alpha * (config_.max_lookahead_distance - config_.min_lookahead_distance);
    return std::clamp(lookahead, config_.min_lookahead_distance, config_.max_lookahead_distance);
  }

  bool VelocityPlanner::computeTarget(double current_x_m, double current_y_m, double current_yaw,
                                      double& target_vx_m, double& target_vy_m, double& target_wz) {
    if (path_.poses.size() < 2) {
      return false;
    }

    const auto&  goal_pose    = path_.poses.back();
    const double goal_x       = goal_pose.x;
    const double goal_y       = goal_pose.y;
    const double goal_yaw     = goal_pose.yaw;
    const double dx_goal      = goal_x - current_x_m;
    const double dy_goal      = goal_y - current_y_m;
    const double dist_to_goal = std::hypot(dx_goal, dy_goal);

    if (dist_to_goal <= config_.goal_position_tolerance) {
      const double yaw_err_goal = normalizeAngle(goal_yaw - current_yaw);
      if (std::fabs(yaw_err_goal) <= config_.goal_yaw_tolerance) {
        return false;
      } else {
        target_vx_m = 0.0;
        target_vy_m = 0.0;
        target_wz   = std::clamp(config_.heading_kp * yaw_err_goal, -1e9, 1e9);
        return true;
      }
    }

    std::size_t closest_index = 0;
    if (has_last_closest_index_) {
      closest_index   = std::min(last_closest_index_, path_.poses.size() - 1);
      bool progressed = true;
      while (progressed && (closest_index + 1) < path_.poses.size()) {
        const auto&  p_curr  = path_.poses[closest_index];
        const auto&  p_next  = path_.poses[closest_index + 1];
        const double dx_curr = p_curr.x - current_x_m;
        const double dy_curr = p_curr.y - current_y_m;
        const double dx_next = p_next.x - current_x_m;
        const double dy_next = p_next.y - current_y_m;
        const double d2_curr = dx_curr * dx_curr + dy_curr * dy_curr;
        const double d2_next = dx_next * dx_next + dy_next * dy_next;
        if (d2_next < d2_curr) {
          closest_index += 1;
        } else {
          progressed = false;
        }
      }
    } else {
      double closest_dist_sq = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < path_.poses.size(); ++i) {
        const auto&  p  = path_.poses[i];
        const double dx = p.x - current_x_m;
        const double dy = p.y - current_y_m;
        const double d2 = dx * dx + dy * dy;
        if (d2 < closest_dist_sq) {
          closest_dist_sq = d2;
          closest_index   = i;
        }
      }
    }
    last_closest_index_     = closest_index;
    has_last_closest_index_ = true;

    const double lookahead_m = computeDynamicLookahead(closest_index);

    double accumulated     = 0.0;
    double target_x        = path_.poses[closest_index].x;
    double target_y        = path_.poses[closest_index].y;
    double segment_heading = current_yaw;

    for (std::size_t i = closest_index; i + 1 < path_.poses.size(); ++i) {
      const auto&  p0      = path_.poses[i];
      const auto&  p1      = path_.poses[i + 1];
      const double seg_dx  = p1.x - p0.x;
      const double seg_dy  = p1.y - p0.y;
      const double seg_len = std::hypot(seg_dx, seg_dy);
      if (seg_len < 1e-6) {
        continue;
      }
      if (accumulated + seg_len >= lookahead_m) {
        const double remain = lookahead_m - accumulated;
        const double t      = remain / seg_len;
        target_x            = p0.x + t * seg_dx;
        target_y            = p0.y + t * seg_dy;
        segment_heading     = std::atan2(seg_dy, seg_dx);
        break;
      } else {
        accumulated += seg_len;
        target_x        = p1.x;
        target_y        = p1.y;
        segment_heading = std::atan2(seg_dy, seg_dx);
      }
    }

    const double dx   = target_x - current_x_m;
    const double dy   = target_y - current_y_m;
    const double dist = std::hypot(dx, dy);

    if (dist < 1e-6) {
      target_vx_m = 0.0;
      target_vy_m = 0.0;
    } else {
      const double slow_factor = std::clamp(dist_to_goal / config_.slow_down_distance, 0.0, 1.0);
      const double lookahead_ratio =
          std::clamp(lookahead_m / std::max(1e-9, config_.lookahead_distance), 0.0, 2.0);
      const double local_speed =
          std::max(config_.min_target_speed, config_.target_speed * slow_factor * lookahead_ratio);
      const double scale = local_speed / dist;
      target_vx_m        = dx * scale;
      target_vy_m        = dy * scale;
    }

    const double desired_heading =
        (dist_to_goal < config_.final_heading_switch_distance) ? goal_yaw : segment_heading;
    const double yaw_error = normalizeAngle(desired_heading - current_yaw);
    target_wz              = config_.heading_kp * yaw_error;
    return true;
  }

} // namespace gvc