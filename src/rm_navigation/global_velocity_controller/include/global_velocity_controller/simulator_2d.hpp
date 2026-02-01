#pragma once

#include "global_velocity_controller/types.hpp"

#include <random>

namespace gvc {

  struct SimulatorConfig {
    double init_x{0.0};
    double init_y{0.0};
    double init_yaw{0.0};
    double linear_meas_noise_std{0.05};
    double angular_meas_noise_std{0.02};
    double process_linear_noise_std{0.0};
    double process_angular_noise_std{0.0};
    double integration_step_dt{0.02};
    int    noise_seed{42};
  };

  class Simulator2D {
  public:
    Simulator2D();
    explicit Simulator2D(const SimulatorConfig& config);

    void                   setConfig(const SimulatorConfig& config);
    const SimulatorConfig& getConfig() const { return config_; }

    void   setPose(double x, double y, double yaw);
    Pose2D getPose() const { return pose_; }

    // Return measured twist in body frame given last commanded twist (with measurement noise)
    Twist2D sampleMeasuredTwistBody(const Twist2D& last_cmd_body);

    // Integrate body-frame command with process noise and sub-steps
    void integrateBodyCommand(const Twist2D& cmd_body, double dt);

  private:
    SimulatorConfig config_{};
    Pose2D          pose_{};
    std::mt19937    rng_;
  };

} // namespace gvc