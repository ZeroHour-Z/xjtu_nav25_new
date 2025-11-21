#pragma once

#include <cmath>
#include <vector>

namespace gvc { // global velocity controller

  struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Twist2D {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct Path2D {
    std::vector<Pose2D> poses;
  };

  inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

} // namespace gvc