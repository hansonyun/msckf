#include "propagator.h"
#include "params/params.h"
#include "state.h"
#include "utils/common_math.h"
#include "utils/log.h"
#include "utils/sensor_data.h"
namespace msckf {

Propagator::Propagator(const Params &params)
    : imu_buffer_(std::make_shared<std::deque<ImuData>>()) {
  gravity_ << 0.0, 0.0, -params.gravity_mag;
  Q_.setIdentity();
  Q_.block(0, 0, 3, 3) *= params.imu_params.sigma_w_2;
  Q_.block(3, 3, 3, 3) *= params.imu_params.sigma_a_2;
  Q_.block(6, 6, 3, 3) *= params.imu_params.sigma_wb_2;
  Q_.block(9, 9, 3, 3) *= params.imu_params.sigma_ab_2;
}

Propagator::~Propagator() {}

void Propagator::FeedImu(const ImuData &imu) { imu_buffer_->push_back(imu); }

bool Propagator::Propagate(uint64_t timestamp, State &state) {

  // 获取imu
  std::vector<ImuData> imu_vec;
  if (!GetImuData(state.timestamp_, timestamp, imu_vec)) {
    return false;
  }

  ACHECK(imu_vec.size() > 2) << "Error get wrong num imu!";
  ACHECK(imu_vec.front().timestamp < state.timestamp_) << "Error get start imu!";
  ACHECK(state.timestamp_ == imu_vec[1].timestamp) << "Error get ts imu!";
  ACHECK(timestamp == imu_vec.back().timestamp) << "Error get te imu!";

  // 零偏补偿
  for (auto &imu_data : imu_vec) {
    imu_data.am -= state.ba_;
    imu_data.wm -= state.bg_;
  }

  // 传播状态
  StatePropagate(imu_vec, state);

  return true;
}

bool Propagator::GetImuData(uint64_t ts, uint64_t te, std::vector<ImuData> &imu_vec) {

  // 校验输入及输出的有效性
  if (ts > te) {
    AWARN << "ts > te" << ts << "," << te;
    return false;
  }

  if (imu_buffer_->empty()) {
    AWARN << "imu_buffer_ is empty";
    return false;
  }

  if (imu_buffer_->back().timestamp < te || imu_buffer_->front().timestamp > ts) {
    AWARN << "imu_buffer_ is not enough " << imu_buffer_->back().timestamp << "," << te;
    return false;
  }

  // 移除ts之前的所有数据, 保留ts之前的一帧
  auto it = imu_buffer_->begin();
  while (it != imu_buffer_->end()) {
    if (it->timestamp >= ts) {
      break;
    }
    it++;
  }
  imu_buffer_->erase(imu_buffer_->begin(), std::prev(it));

  // 对ts到t_e之间的数据进行处理
  it = imu_buffer_->begin();
  auto it_next = std::next(it);
  while (it_next != imu_buffer_->end()) {

    if (it_next->timestamp >= ts && it->timestamp < ts) {
      imu_vec.push_back(*it);
      imu_vec.push_back(ImuData::interpolate_data(*it, *it_next, ts));
    }

    if (it->timestamp > ts && it->timestamp < te && it_next->timestamp > ts &&
        it_next->timestamp < te) {
      imu_vec.push_back(*it);
    }

    if (it_next->timestamp >= te && it->timestamp < te) {
      imu_vec.push_back(ImuData::interpolate_data(*it, *it_next, te));
      break;
    }
    it++;
    it_next++;
  }
  return true;
}

void Propagator::StatePropagate(const std::vector<ImuData> &imu_vec, State &state) {
  Eigen::Matrix<double, State::kDim, State::kDim> Phi_sum;
  Phi_sum.setIdentity();
  Eigen::Matrix<double, State::kDim, State::kDim> Q_sum;
  Q_sum.setZero();

  for (size_t i = 2; i < imu_vec.size(); ++i) {
    double dt = (imu_vec[i].timestamp - imu_vec[i - 1].timestamp) * 1e-9;
    double dt_2 = std::pow(dt, 2);

    // 计算 delta_q及delta_R
    auto wm_dt = CalcEquivalentAxisVector(imu_vec[i - 2], imu_vec[i - 1], imu_vec[i], state.bg_);
    Eigen::Quaterniond delta_q_inv(Eigen::AngleAxisd(wm_dt.norm(), wm_dt.normalized()));
    Eigen::Quaterniond delta_q = delta_q_inv.inverse();
    Eigen::Matrix3d delta_R_inv = delta_q_inv.toRotationMatrix();
    Eigen::Matrix3d delta_R = delta_q.toRotationMatrix();
    Eigen::Matrix3d prev_R = state.q_.toRotationMatrix();
    Eigen::Matrix3d prev_R_trans = prev_R.transpose();
    Eigen::Matrix3d curr_R = delta_R * prev_R;
    Eigen::Matrix3d curr_R_trans = curr_R.transpose();

    // 计算 delta_v
    Eigen::Vector3d s_ell =
        0.5 * (imu_vec[i - 1].am + delta_q_inv * imu_vec[i].am - 2 * state.ba_) * dt;
    Eigen::Vector3d delta_v = gravity_ * dt + prev_R_trans * s_ell;

    // 计算 delta_p
    Eigen::Vector3d y_ell = 0.5 * s_ell * dt;
    Eigen::Vector3d delta_p = 0.5 * gravity_ * dt_2 + prev_R_trans * y_ell;

    // 更新Phi_sum, 参考论文推导 4.4.4 节
    Eigen::Matrix<double, State::kDim, State::kDim> Phi_ell;
    Phi_ell.setIdentity();
    Eigen::Matrix3d rot_mat_inter = 0.5 * dt * (prev_R_trans + curr_R_trans);
    Phi_ell.block(0, 9, 3, 3) = -rot_mat_inter;
    Phi_ell.block(6, 0, 3, 3) = -skew_x(delta_v - gravity_ * dt);
    Phi_ell.block(6, 9, 3, 3) = skew_x(curr_R_trans * imu_vec[i].am) * rot_mat_inter * dt * 0.5;
    Phi_ell.block(6, 12, 3, 3) = -rot_mat_inter;
    Phi_ell.block(3, 0, 3, 3) = -skew_x(delta_p - state.v_ * dt - 0.5 * gravity_ * dt_2);
    Phi_ell.block(3, 6, 3, 3) = dt * Eigen::Matrix3d::Identity();
    Phi_ell.block(3, 9, 3, 3) = Phi_ell.block(6, 9, 3, 3) * dt * 0.5;
    Phi_ell.block(3, 12, 3, 3) = Phi_ell.block(6, 12, 3, 3) * dt * 0.5;
    Phi_sum = Phi_ell * Phi_sum;

    AWARN << "Phi is \n" << Phi_ell;

    // 更新Q_sum, 参考论文推导 4.4.6节
    Eigen::Matrix<double, State::kDim, 12> G_ell;
    G_ell.setZero();
    G_ell.block(0, 0, 3, 3) = -prev_R_trans;
    G_ell.block(0, 6, 3, 3) = -prev_R_trans * dt;
    G_ell.block(3, 3, 3, 3) = -prev_R_trans * dt;
    G_ell.block(6, 0, 3, 3) = prev_R_trans * skew_x(imu_vec[i - 1].am) * dt;
    G_ell.block(6, 3, 3, 3) = -prev_R_trans;
    G_ell.block(6, 9, 3, 3) = -prev_R_trans * dt;
    G_ell.block(9, 6, 3, 3).setIdentity();
    G_ell.block(12, 9, 3, 3).setIdentity();
    Q_sum = Phi_ell * Q_sum * Phi_ell.transpose() + G_ell * Q_ * dt * G_ell.transpose();

    // 更新状态
    state.timestamp_ = imu_vec[i].timestamp;
    state.q_ = delta_q * state.q_;
    state.v_ += delta_v;
    state.p_ += delta_p;
  }
  state.cov_ = Phi_sum * state.cov_ * Phi_sum.transpose() + Q_sum;
  return;
}

Eigen::Vector3d Propagator::CalcEquivalentAxisVector(const ImuData &pre_pre_imu,
                                                     const ImuData &pre_imu,
                                                     const ImuData &curr_imu,
                                                     const Eigen::Vector3d &bg) {

  double delta_t = (curr_imu.timestamp - pre_imu.timestamp) * 1e-9;
  double pre_delta_t = (pre_imu.timestamp - pre_pre_imu.timestamp) * 1e-9;

  Eigen::Vector3d wm_dt = 0.5 * (pre_imu.wm + curr_imu.wm - 2 * bg) * delta_t;
  Eigen::Vector3d pre_wm_dt = 0.5 * (pre_pre_imu.wm + pre_imu.wm - 2 * bg) * pre_delta_t;

  Eigen::Vector3d cone_effect = 1.0 / 12.0 * pre_wm_dt.cross(wm_dt);

  wm_dt = wm_dt + cone_effect;

  return wm_dt;
}

} // namespace msckf