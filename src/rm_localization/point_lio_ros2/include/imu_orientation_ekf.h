#pragma once

#include <deque>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

class ImuOrientationEkf {
public:
    struct HistoryEntry {
        double stamp{0.0};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    };

    ImuOrientationEkf();

    void configure(double orientation_process_variance,
                   double angular_velocity_process_variance,
                   double gyro_measurement_variance,
                   double orientation_measurement_variance,
                   double max_history_duration_seconds);

    void reset(const Eigen::Quaterniond& orientation,
               const Eigen::Vector3d& angular_velocity,
               double timestamp);

    void processImuSample(double timestamp, const Eigen::Vector3d& gyro);

    void updateWithOrientation(double timestamp, const Eigen::Quaterniond& orientation);

    bool alignWithLidar(double lidar_timestamp, const Eigen::Quaterniond& lidar_orientation);

    bool hasEstimate() const { return initialized_; }
    double lastTimestamp() const { return last_timestamp_; }

    Eigen::Vector3d euler() const;
    Eigen::Quaterniond quaternion() const;
    Eigen::Vector3d angularVelocity() const { return state_.segment<3>(3); }

    void setHistoryDuration(double duration_seconds) { max_history_duration_ = duration_seconds; }

private:
    static double wrapAngle(double angle);
    static Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& rpy);
    static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q);

    void predict(double dt);
    void updateAngularVelocity(const Eigen::Vector3d& gyro);
    void applyOrientationMeasurement(const Eigen::Vector3d& euler, double timestamp);
    void recordHistory(double timestamp);
    void pruneHistory(double min_timestamp);

    std::deque<HistoryEntry> history_;
    Eigen::Matrix<double, 6, 1> state_;
    Eigen::Matrix<double, 6, 6> covariance_;
    Eigen::Matrix3d orientation_process_noise_;
    Eigen::Matrix3d angular_velocity_process_noise_;
    Eigen::Matrix3d gyro_measurement_noise_;
    Eigen::Matrix3d orientation_measurement_noise_;

    double max_history_duration_;
    double last_timestamp_;
    bool initialized_;
};

ImuOrientationEkf& GetOrientationEkf();
std::mutex& GetOrientationEkfMutex();

