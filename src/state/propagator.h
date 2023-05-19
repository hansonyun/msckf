#pragma once
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>
namespace msckf {
class ImuData;
class State;
class Params;

class Propagator {

public:
  Propagator(const Params &nm);
  ~Propagator();

  void FeedImu(const ImuData &imu);

  bool Propagate(uint64_t timestamp, State &state);

private:
  bool GetImuData(uint64_t ts, uint64_t te, std::vector<ImuData> &imu_vec);

  void StatePropagate(const std::vector<ImuData> &imu_vec, State &state);

  Eigen::Vector3d CalcEquivalentAxisVector(const ImuData &pre_pre_imu, const ImuData &pre_imu,
                                           const ImuData &curr_imu, const Eigen::Vector3d &wb);

  std::shared_ptr<std::deque<ImuData>> imu_buffer_;

  Eigen::Vector3d gravity_;

  Eigen::Matrix<double, 12, 12> Q_ = Eigen::Matrix<double, 12, 12>::Identity();
};
} // namespace msckf
