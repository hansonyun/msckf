#include "params/params.h"
#include "sim/simulator.h"
#include "state/propagator.h"
#include "state/state.h"
#include "utils/log.h"
#include "utils/sensor_data.h"
#include <iostream>
#include <thread>
#include <utility>

void run_once(size_t count, msckf::State &state, Eigen::Vector<double, 17> &true_state) {
  msckf::Params params;
  params.sim_params.sim_seed_measurements = count;
  msckf::Simulator sim(params);
  msckf::Propagator propagator(params);
  int imu_count = 0;
  while (imu_count < 800 && sim.ok()) {

    double time_imu;
    Eigen::Vector3d wm, am;
    if (sim.get_next_imu(time_imu, wm, am)) {
      msckf::ImuData imu;
      imu.timestamp = time_imu * 1e9;
      imu.wm = wm;
      imu.am = am;
      propagator.FeedImu(imu);
    }

    if (!sim.get_state(time_imu, true_state)) {
      AERROR << "This should not happen!";
    }

    if (imu_count == 2) {
      state.timestamp_ = true_state[0] * 1e9;
      state.q_.coeffs() = true_state.segment<4>(1);
      state.p_ = true_state.segment<3>(5);
      state.v_ = true_state.segment<3>(8);
      state.bg_ = true_state.segment<3>(11);
      state.ba_ = true_state.segment<3>(14);
      state.cov_.setZero();
    }

    if (imu_count > 2) {
      if (propagator.Propagate(time_imu * 1e9, state)) {
        AWARN << "---------------------------------------";
        AWARN << "state.timestamp_ = " << state.timestamp_ << " vs "
              << static_cast<uint64_t>(true_state[0] * 1e9);
        AWARN << "state.q_ = " << state.q_.coeffs().transpose() << " vs "
              << true_state.segment<4>(1).transpose();
        AWARN << "state.p_ = " << state.p_.transpose() << " vs "
              << true_state.segment<3>(5).transpose();
        AWARN << "state.v_ = " << state.v_.transpose() << " vs "
              << true_state.segment<3>(8).transpose();
        AWARN << "state.bg_ = " << state.bg_.transpose() << " vs "
              << true_state.segment<3>(11).transpose();
        AWARN << "state.ba_ = " << state.ba_.transpose() << " vs "
              << true_state.segment<3>(14).transpose();
        AWARN << "state.cov_ = \n" << state.cov_;
      }
    }

    imu_count++;
  }
  return;
}

int main(int argc, char **argv) {

  std::vector<std::pair<msckf::State, Eigen::Vector<double, 17>>> states;
  states.resize(500);
  msckf::State state;
  Eigen::Vector<double, 17> true_state;

  // 获取初始state
  msckf::Params params;
  msckf::Simulator sim(params);
  for (size_t i = 0; i < 3; i++) {
    double time_imu;
    Eigen::Vector3d wm, am;
    sim.get_next_imu(time_imu, wm, am);
    if (!sim.get_state(time_imu, true_state)) {
      AERROR << "This should not happen!";
    }
  }
  state.timestamp_ = true_state[0] * 1e9;
  state.q_.coeffs() = true_state.segment<4>(1);
  state.p_ = true_state.segment<3>(5);
  state.v_ = true_state.segment<3>(8);
  state.bg_ = true_state.segment<3>(11);
  state.ba_ = true_state.segment<3>(14);
  state.cov_.setZero();

  for (size_t i = 0; i < states.size(); ++i) {
    states[i].first = state;
    states[i].second = true_state;
  }

  // 500
  std::vector<std::thread> threads;
  for (size_t i = 0; i < states.size(); ++i) {
    threads.emplace_back(
        std::thread(run_once, std::ref(i), std::ref(states[i].first), std::ref(states[i].second)));
  }
  for (size_t i = 0; i < states.size(); ++i) {
    if (threads[i].joinable())
      threads[i].join();
  }

  Eigen::Matrix<double, 15, 15> cov;
  cov.setZero();
  for (size_t i = 0; i < states.size(); ++i) {
    Eigen::Vector<double, 15> diff;
    auto delta_aa = Eigen::AngleAxisd(Eigen::Quaterniond(states[i].second.segment<4>(1)).inverse() *
                                      states[i].first.q_);

    diff.segment<3>(0) = delta_aa.axis() * delta_aa.angle();
    diff.segment<3>(3) = states[i].second.segment<3>(5) - states[i].first.p_;
    diff.segment<3>(6) = states[i].second.segment<3>(8) - states[i].first.v_;
    diff.segment<3>(9) = states[i].second.segment<3>(11) - states[i].first.bg_;
    diff.segment<3>(12) = states[i].second.segment<3>(14) - states[i].first.ba_;

    cov += diff * diff.transpose();
  }
  cov /= states.size();

  Eigen::Matrix<double, 15, 15> est_cov;
  est_cov.setZero();
  for (size_t i = 0; i < states.size(); ++i) {
    est_cov += states[i].first.cov_;
  }
  est_cov /= states.size();

  std::cout << "cov = \n" << cov << std::endl;
  std::cout << "est_cov = \n" << est_cov << std::endl;
  return 0;
}