#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace msckf {
struct State {
  static const size_t kDim = 15;
  static const size_t kThetaIdx = 0;
  static const size_t kPosIdx = 3;
  static const size_t kVelIdx = 6;
  static const size_t kBgIdx = 9;
  static const size_t kBaIdx = 12;
  static const size_t kThetaSize = 3;
  static const size_t kPosSize = 3;
  static const size_t kVelSize = 3;
  static const size_t kBgSize = 3;
  static const size_t kBaSize = 3;

  uint64_t timestamp_;

  // 顺序为 q,p,v,bg,ba
  Eigen::Quaterniond q_;
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
  Eigen::Matrix<double, kDim, kDim> cov_;
};

} // namespace msckf
