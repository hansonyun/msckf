#pragma once
#include <Eigen/Dense>
namespace msckf {

/**
 * @brief Struct for a single imu measurement (time, wm, am)
 */
struct ImuData {

  /// Timestamp of the reading
  uint64_t timestamp;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> wm;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Matrix<double, 3, 1> am;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp; }

  /// Nice helper function that will linearly interpolate between two imu messages.
  static ImuData interpolate_data(const ImuData &imu_1, const ImuData &imu_2, uint64_t timestamp) {
    // time-distance lambda
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    // ADEBUG << "lambda - " << lambda;
    // interpolate between the two times
    ImuData data;
    data.timestamp = timestamp;
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }
};
} // namespace msckf