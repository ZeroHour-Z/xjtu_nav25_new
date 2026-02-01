#include "global_velocity_controller/simulator_2d.hpp"

#include <algorithm>
#include <cmath>

namespace gvc {

  Simulator2D::Simulator2D() {
    rng_.seed(static_cast<uint32_t>(config_.noise_seed));
    pose_.x   = config_.init_x;
    pose_.y   = config_.init_y;
    pose_.yaw = config_.init_yaw;
  }

  Simulator2D::Simulator2D(const SimulatorConfig& config) : config_(config) {
    rng_.seed(static_cast<uint32_t>(config_.noise_seed));
    pose_.x   = config_.init_x;
    pose_.y   = config_.init_y;
    pose_.yaw = config_.init_yaw;
  }

  void Simulator2D::setConfig(const SimulatorConfig& config) {
    config_ = config;
    rng_.seed(static_cast<uint32_t>(config_.noise_seed));
  }

  void Simulator2D::setPose(double x, double y, double yaw) {
    pose_.x   = x;
    pose_.y   = y;
    pose_.yaw = yaw;
  }

  Twist2D Simulator2D::sampleMeasuredTwistBody(const Twist2D& last_cmd_body) {
    std::normal_distribution<double> lin_noise(0.0, config_.linear_meas_noise_std);
    std::normal_distribution<double> ang_noise(0.0, config_.angular_meas_noise_std);

    Twist2D measured{};
    measured.vx = last_cmd_body.vx + lin_noise(rng_);
    measured.vy = last_cmd_body.vy + lin_noise(rng_);
    measured.wz = last_cmd_body.wz + ang_noise(rng_);
    return measured;
  }

  void Simulator2D::integrateBodyCommand(const Twist2D& cmd_body, double dt) {
    std::normal_distribution<double> proc_lin_noise(0.0, config_.process_linear_noise_std);
    std::normal_distribution<double> proc_ang_noise(0.0, config_.process_angular_noise_std);
    const double                     nx_total = proc_lin_noise(rng_);
    const double                     ny_total = proc_lin_noise(rng_);
    const double                     nw_total = proc_ang_noise(rng_);

    double remaining = dt;
    while (remaining > 1e-9) {
      const double step = std::min(remaining, config_.integration_step_dt);
      const double frac = (dt > 1e-12) ? (step / dt) : 1.0;
      const double nx   = nx_total * frac;
      const double ny   = ny_total * frac;
      const double nw   = nw_total * frac;

      const double c        = std::cos(pose_.yaw);
      const double s        = std::sin(pose_.yaw);
      const double vx_m_cmd = c * cmd_body.vx - s * cmd_body.vy;
      const double vy_m_cmd = s * cmd_body.vx + c * cmd_body.vy;

      pose_.x += (vx_m_cmd + nx) * step;
      pose_.y += (vy_m_cmd + ny) * step;
      pose_.yaw = normalizeAngle(pose_.yaw + (cmd_body.wz + nw) * step);

      remaining -= step;
    }
  }

} // namespace gvc