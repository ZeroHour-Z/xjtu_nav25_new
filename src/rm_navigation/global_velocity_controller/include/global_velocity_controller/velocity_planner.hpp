#pragma once

#include "global_velocity_controller/types.hpp"

#include <algorithm>
#include <cstddef>

namespace gvc {

  struct VelocityPlannerConfig {
    double lookahead_distance{0.5};
    double target_speed{0.6};
    double heading_kp{1.0};
    double slow_down_distance{0.6};
    double min_target_speed{0.03};
    double goal_position_tolerance{0.06};
    double goal_yaw_tolerance{0.06};
    double final_heading_switch_distance{0.3};
    bool   enable_dynamic_lookahead{true};
    double min_lookahead_distance{0.3};
    double max_lookahead_distance{1.0};
    double curvature_window_distance{1.0};
    double curvature_low{0.05};
    double curvature_high{0.8};
  };

  class VelocityPlanner {
  public:
    VelocityPlanner() = default;
    explicit VelocityPlanner(const VelocityPlannerConfig& config);

    void                         setConfig(const VelocityPlannerConfig& config);
    const VelocityPlannerConfig& getConfig() const { return config_; }

    void setPath(const Path2D& path);
    bool hasPath() const { return !path_.poses.empty(); }
    void resetTracking();

    // Compute target velocities (map frame) and yaw rate based on current pose
    bool computeTarget(double current_x_m, double current_y_m, double current_yaw,
                       double& target_vx_m, double& target_vy_m, double& target_wz);

  private:
    double computeDynamicLookahead(std::size_t start_index) const;

    VelocityPlannerConfig config_{};
    Path2D                path_{};
    bool                  has_last_closest_index_{false};
    std::size_t           last_closest_index_{0};
  };

} // namespace gvc