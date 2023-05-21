#pragma once

#include "utils/log.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
namespace msckf {

struct SimParams {

  /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation
  /// to generate the same true measurements, but different noise values.
  int sim_seed_measurements = 0;

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw)
  /// format.
  std::string sim_traj_path = "data/sim/udel_arl.txt";

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;
};

} // namespace msckf
