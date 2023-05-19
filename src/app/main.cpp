#include "params/params.h"
#include "sim/simulator.h"
#include "state/propagator.h"
#include "state/state.h"
#include "utils/log.h"
#include "utils/sensor_data.h"
#include <iostream>
int main(int argc, char **argv) {
  msckf::Params params;
  msckf::Simulator sim(params);
  msckf::State state;
  msckf::Propagator propagator(params);
  int imu_count = 0;
  while (sim.ok()) {

    double time_imu;
    Eigen::Vector3d wm, am;
    if (sim.get_next_imu(time_imu, wm, am)) {
      msckf::ImuData imu;
      imu.timestamp = time_imu * 1e9;
      imu.wm = wm;
      imu.am = am;
      propagator.FeedImu(imu);
    }

    Eigen::Vector<double, 17> true_state;
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

  return 0;
}