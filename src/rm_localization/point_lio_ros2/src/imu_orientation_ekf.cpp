#include "imu_orientation_ekf.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kDefaultOrientationProcessVariance = 1e-6;
constexpr double kDefaultAngularVelocityProcessVariance = 1e-4;
constexpr double kDefaultGyroMeasurementVariance = 1e-4;
constexpr double kDefaultOrientationMeasurementVariance = 1e-4;
constexpr double kDefaultHistoryDurationSeconds = 2.0;
constexpr double kTimeEqualityEps = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
} // namespace

ImuOrientationEkf::ImuOrientationEkf():
    state_(Eigen::Matrix<double, 6, 1>::Zero()),
    covariance_(Eigen::Matrix<double, 6, 6>::Identity()),
    orientation_process_noise_(Eigen::Matrix3d::Identity() * kDefaultOrientationProcessVariance),
    angular_velocity_process_noise_(
        Eigen::Matrix3d::Identity() * kDefaultAngularVelocityProcessVariance
    ),
    gyro_measurement_noise_(Eigen::Matrix3d::Identity() * kDefaultGyroMeasurementVariance),
    orientation_measurement_noise_(
        Eigen::Matrix3d::Identity() * kDefaultOrientationMeasurementVariance
    ),
    max_history_duration_(kDefaultHistoryDurationSeconds),
    last_timestamp_(0.0),
    initialized_(false) {}

void ImuOrientationEkf::configure(
    double orientation_process_variance,
    double angular_velocity_process_variance,
    double gyro_measurement_variance,
    double orientation_measurement_variance,
    double max_history_duration_seconds
) {
    orientation_process_noise_ = Eigen::Matrix3d::Identity() * orientation_process_variance;
    angular_velocity_process_noise_ =
        Eigen::Matrix3d::Identity() * angular_velocity_process_variance;
    gyro_measurement_noise_ = Eigen::Matrix3d::Identity() * gyro_measurement_variance;
    orientation_measurement_noise_ = Eigen::Matrix3d::Identity() * orientation_measurement_variance;
    if (max_history_duration_seconds > 0.01) {
        max_history_duration_ = max_history_duration_seconds;
    }
}

void ImuOrientationEkf::reset(
    const Eigen::Quaterniond& orientation,
    const Eigen::Vector3d& angular_velocity,
    double timestamp
) {
    state_.segment<3>(0) = quaternionToEuler(orientation);
    state_.segment<3>(3) = angular_velocity;
    covariance_.setIdentity();
    covariance_ *= 1e-3;
    last_timestamp_ = timestamp;
    initialized_ = true;
    history_.clear();
    recordHistory(timestamp);
}

void ImuOrientationEkf::processImuSample(double timestamp, const Eigen::Vector3d& gyro) {
    if (!initialized_) {
        reset(Eigen::Quaterniond::Identity(), gyro, timestamp);
        return;
    }

    const double dt = std::max(0.0, timestamp - last_timestamp_);
    predict(dt);
    updateAngularVelocity(gyro);
    last_timestamp_ = timestamp;
    recordHistory(timestamp);
}

void ImuOrientationEkf::updateWithOrientation(
    double timestamp,
    const Eigen::Quaterniond& orientation
) {
    if (!initialized_) {
        reset(orientation, Eigen::Vector3d::Zero(), timestamp);
        return;
    }

    applyOrientationMeasurement(quaternionToEuler(orientation), timestamp);
    recordHistory(last_timestamp_);
}

bool ImuOrientationEkf::alignWithLidar(
    double lidar_timestamp,
    const Eigen::Quaterniond& lidar_orientation
) {
    if (!initialized_) {
        reset(lidar_orientation, Eigen::Vector3d::Zero(), lidar_timestamp);
        return true;
    }
    if (history_.empty()) {
        updateWithOrientation(lidar_timestamp, lidar_orientation);
        return false;
    }

    auto closest_it = history_.begin();
    double min_diff = std::abs(history_.front().stamp - lidar_timestamp);
    for (auto it = history_.begin(); it != history_.end(); ++it) {
        double diff = std::abs(it->stamp - lidar_timestamp);
        if (diff < min_diff) {
            min_diff = diff;
            closest_it = it;
        }
    }

    Eigen::Quaterniond current_orientation = quaternion();
    Eigen::Quaterniond delta = closest_it->orientation.conjugate() * current_orientation;
    Eigen::Quaterniond corrected_current = lidar_orientation * delta;
    Eigen::Quaterniond correction = corrected_current * current_orientation.conjugate();

    state_.segment<3>(0) = quaternionToEuler(corrected_current);

    for (auto it = closest_it; it != history_.end(); ++it) {
        it->orientation = correction * it->orientation;
    }

    // Ensure there is a history entry at the lidar timestamp.
    auto insert_pos = history_.begin();
    while (insert_pos != history_.end() && insert_pos->stamp < lidar_timestamp - kTimeEqualityEps) {
        ++insert_pos;
    }
    if (insert_pos != history_.end()
        && std::abs(insert_pos->stamp - lidar_timestamp) <= kTimeEqualityEps)
    {
        insert_pos->orientation = lidar_orientation;
    } else {
        history_.insert(insert_pos, HistoryEntry { lidar_timestamp, lidar_orientation });
    }

    pruneHistory(std::max(last_timestamp_, lidar_timestamp) - max_history_duration_);
    return true;
}

Eigen::Vector3d ImuOrientationEkf::euler() const {
    return state_.segment<3>(0);
}

Eigen::Quaterniond ImuOrientationEkf::quaternion() const {
    return eulerToQuaternion(state_.segment<3>(0));
}

double ImuOrientationEkf::wrapAngle(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

Eigen::Quaterniond ImuOrientationEkf::eulerToQuaternion(const Eigen::Vector3d& rpy) {
    Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
    return yaw * pitch * roll;
}

Eigen::Vector3d ImuOrientationEkf::quaternionToEuler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    euler.x() = wrapAngle(euler.x());
    euler.y() = wrapAngle(euler.y());
    euler.z() = wrapAngle(euler.z());
    return euler;
}

void ImuOrientationEkf::predict(double dt) {
    if (dt <= 0.0) {
        return;
    }

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    state_ = F * state_;
    for (int i = 0; i < 3; ++i) {
        state_(i) = wrapAngle(state_(i));
    }

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.block<3, 3>(0, 0) = orientation_process_noise_ * dt;
    Q.block<3, 3>(3, 3) = angular_velocity_process_noise_ * dt;

    covariance_ = F * covariance_ * F.transpose() + Q;
}

void ImuOrientationEkf::updateAngularVelocity(const Eigen::Vector3d& gyro) {
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d innovation = gyro - state_.segment<3>(3);
    Eigen::Matrix3d S = H * covariance_ * H.transpose() + gyro_measurement_noise_;
    Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();

    state_ += K * innovation;
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    covariance_ = (I - K * H) * covariance_;
}

void ImuOrientationEkf::applyOrientationMeasurement(
    const Eigen::Vector3d& euler,
    double timestamp
) {
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d innovation = euler - state_.segment<3>(0);
    for (int i = 0; i < 3; ++i) {
        innovation(i) = wrapAngle(innovation(i));
    }

    Eigen::Matrix3d S = H * covariance_ * H.transpose() + orientation_measurement_noise_;
    Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();

    state_ += K * innovation;
    for (int i = 0; i < 3; ++i) {
        state_(i) = wrapAngle(state_(i));
    }

    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    covariance_ = (I - K * H) * covariance_;
    last_timestamp_ = std::max(last_timestamp_, timestamp);
}

void ImuOrientationEkf::recordHistory(double timestamp) {
    Eigen::Quaterniond orientation = quaternion();
    if (!history_.empty() && std::abs(history_.back().stamp - timestamp) <= kTimeEqualityEps) {
        history_.back().orientation = orientation;
    } else {
        history_.push_back(HistoryEntry { timestamp, orientation });
    }

    pruneHistory(timestamp - max_history_duration_);
}

void ImuOrientationEkf::pruneHistory(double min_timestamp) {
    while (history_.size() > 1 && history_.front().stamp < min_timestamp) {
        history_.pop_front();
    }
}

ImuOrientationEkf& GetOrientationEkf() {
    static ImuOrientationEkf instance;
    return instance;
}

std::mutex& GetOrientationEkfMutex() {
    static std::mutex mtx;
    return mtx;
}
