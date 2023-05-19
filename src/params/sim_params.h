#pragma once

#include "utils/log.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
namespace msckf {

struct SimParams {

  /// Seed for initial states (i.e. random feature 3d positions in the generated map)
  int sim_seed_state_init = 0;

  /// Seed for calibration perturbations. Change this to perturb by different random values if
  /// perturbations are enabled.
  int sim_seed_perturb = 0;

  /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation
  /// to generate the same true measurements, but different noise values.
  int sim_seed_measurements = 0;

  /// If we should perturb the calibration that the estimator starts with
  bool sim_do_perturbation = false;

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw)
  /// format.
  std::string sim_traj_path = "data/sim/udel_arl.txt";

  /// We will start simulating after we have moved this much along the b-spline. This prevents
  /// static starts as we init from ground truth in simulation.
  double sim_distance_threshold = 1.2;

  /// Frequency (Hz) that we will simulate our cameras
  double sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// Feature distance we generate features from (minimum)
  double sim_min_feature_gen_distance = 5;

  /// Feature distance we generate features from (maximum)
  double sim_max_feature_gen_distance = 10;

  // /**
  //  * @brief This function will load print out all simulated parameters.
  //  * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
  //  *
  //  * @param parser If not null, this parser will be used to load our parameters
  //  */
  // void print_and_load_simulation(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
  //   if (parser != nullptr) {
  //     parser->parse_config("sim_seed_state_init", sim_seed_state_init);
  //     parser->parse_config("sim_seed_perturb", sim_seed_perturb);
  //     parser->parse_config("sim_seed_measurements", sim_seed_measurements);
  //     parser->parse_config("sim_do_perturbation", sim_do_perturbation);
  //     parser->parse_config("sim_traj_path", sim_traj_path);
  //     parser->parse_config("sim_distance_threshold", sim_distance_threshold);
  //     parser->parse_config("sim_freq_cam", sim_freq_cam);
  //     parser->parse_config("sim_freq_imu", sim_freq_imu);
  //     parser->parse_config("sim_min_feature_gen_dist", sim_min_feature_gen_distance);
  //     parser->parse_config("sim_max_feature_gen_dist", sim_max_feature_gen_distance);
  //   }
  // ADEBUG << "SIMULATION PARAMETERS:";
  // AWARN << "  - state init seed: " << sim_seed_state_init;
  // AWARN << "  - perturb seed: " << sim_seed_perturb;
  // AWARN << "  - measurement seed: " << sim_seed_measurements;
  // AWARN << "  - do perturb?: " << sim_do_perturbation;
  // ADEBUG << "  - traj path: " << sim_traj_path;
  // ADEBUG << "  - dist thresh: " << sim_distance_threshold;
  // ADEBUG << "  - cam feq: " << sim_freq_cam;
  // ADEBUG << "  - imu feq: " << sim_freq_imu;
  // ADEBUG << "  - min feat dist: " << sim_min_feature_gen_distance;
  // ADEBUG << "  - max feat dist: " << sim_max_feature_gen_distance;
  // }
};

} // namespace msckf
