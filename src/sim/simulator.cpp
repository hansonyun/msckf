

#include "simulator.h"
#include "bspline_se3.h"
#include "params/params.h"
#include "utils/dataset_reader.h"
#include "utils/log.h"
#include <chrono>
#include <thread>
namespace msckf {

Simulator::Simulator(const Params &params_) {
  //===============================================================
  // Store a copy of our params
  params = std::make_shared<Params>(params_);

  //===============================================================
  // Load the ground truth trajectory and its spline
  DatasetReader::load_simulated_trajectory(params->sim_params.sim_traj_path, traj_data);
  spline = std::make_shared<BsplineSE3>();
  spline->feed_trajectory(traj_data);

  // Set all our timestamps as starting from the minimum spline time
  timestamp = spline->get_start_time();
  timestamp_last_imu = spline->get_start_time();

  // Get the pose at the current time step
  Eigen::Matrix3d R_GtoI_init;
  Eigen::Vector3d p_IinG_init;
  bool success_pose_init = spline->get_pose(timestamp, R_GtoI_init, p_IinG_init);
  if (!success_pose_init) {
    AERROR << "[SIM]: unable to find the first pose in the spline...\n";
    std::exit(EXIT_FAILURE);
  }

  // Append the current true bias to our history
  hist_true_bias_time.push_back(timestamp_last_imu - 1.0 / params->sim_params.sim_freq_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);
  hist_true_bias_time.push_back(timestamp_last_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);
  hist_true_bias_time.push_back(timestamp_last_imu + 1.0 / params->sim_params.sim_freq_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);

  // Our simulation is running
  is_running = true;

  //===============================================================
  // Load the seeds for the random number generators
  gen_meas_imu = std::mt19937(params->sim_params.sim_seed_measurements);
  gen_meas_imu.seed(params->sim_params.sim_seed_measurements);

  //===============================================================
  // Nice sleep so the user can look at the printout
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

bool Simulator::get_state(double desired_time, Eigen::Matrix<double, 17, 1> &imu_state) {

  // Set to default state
  imu_state.setZero();
  imu_state(4) = 1;

  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG;

  // Get the pose, velocity, and acceleration
  bool success_vel = spline->get_velocity(desired_time, R_GtoI, p_IinG, w_IinI, v_IinG);

  // Find the bounding bias values
  bool success_bias = false;
  size_t id_loc = 0;
  for (size_t i = 0; i < hist_true_bias_time.size() - 1; i++) {
    if (hist_true_bias_time.at(i) < desired_time && hist_true_bias_time.at(i + 1) >= desired_time) {
      id_loc = i;
      success_bias = true;
      break;
    }
  }

  // If failed, then that means we don't have any more spline or bias
  if (!success_vel || !success_bias) {
    return false;
  }

  // Interpolate our biases (they will be at every IMU time step)
  double lambda = (desired_time - hist_true_bias_time.at(id_loc)) /
                  (hist_true_bias_time.at(id_loc + 1) - hist_true_bias_time.at(id_loc));
  Eigen::Vector3d true_bg_interp =
      (1 - lambda) * hist_true_bias_gyro.at(id_loc) + lambda * hist_true_bias_gyro.at(id_loc + 1);
  Eigen::Vector3d true_ba_interp =
      (1 - lambda) * hist_true_bias_accel.at(id_loc) + lambda * hist_true_bias_accel.at(id_loc + 1);

  // Finally lets create the current state
  imu_state(0, 0) = desired_time;
  imu_state.block(1, 0, 4, 1) = Eigen::Quaterniond(R_GtoI).coeffs();
  imu_state.block(5, 0, 3, 1) = p_IinG;
  imu_state.block(8, 0, 3, 1) = v_IinG;
  imu_state.block(11, 0, 3, 1) = true_bg_interp;
  imu_state.block(14, 0, 3, 1) = true_ba_interp;
  return true;
}

bool Simulator::get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am) {

  // Else lets do a new measurement!!!
  timestamp_last_imu += 1.0 / params->sim_params.sim_freq_imu;
  timestamp = timestamp_last_imu;
  time_imu = timestamp_last_imu;

  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

  // Get the pose, velocity, and acceleration
  // NOTE: we get the acceleration between our two IMU
  // NOTE: this is because we are using a constant measurement model for integration
  // bool success_accel = spline->get_acceleration(timestamp+0.5/freq_imu, R_GtoI, p_IinG, w_IinI,
  // v_IinG, alpha_IinI, a_IinG);
  bool success_accel =
      spline->get_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

  // If failed, then that means we don't have any more spline
  // Thus we should stop the simulation
  if (!success_accel) {
    is_running = false;
    return false;
  }

  // Transform omega and linear acceleration into imu frame
  Eigen::Vector3d omega_inI = w_IinI;
  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, params->gravity_mag;
  Eigen::Vector3d accel_inI = R_GtoI * (a_IinG + gravity);

  // Calculate the bias values for this IMU reading
  // NOTE: we skip the first ever bias since we have already appended it
  double dt = 1.0 / params->sim_params.sim_freq_imu;
  std::normal_distribution<double> w(0, 1);
  if (has_skipped_first_bias) {

    // Move the biases forward in time
    true_bias_gyro(0) += params->imu_params.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_gyro(1) += params->imu_params.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_gyro(2) += params->imu_params.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(0) += params->imu_params.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(1) += params->imu_params.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(2) += params->imu_params.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);

    // Append the current true bias to our history
    hist_true_bias_time.push_back(timestamp_last_imu);
    hist_true_bias_gyro.push_back(true_bias_gyro);
    hist_true_bias_accel.push_back(true_bias_accel);
  }
  has_skipped_first_bias = true;

  // Now add noise to these measurements
  wm(0) = omega_inI(0) + true_bias_gyro(0) +
          params->imu_params.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  wm(1) = omega_inI(1) + true_bias_gyro(1) +
          params->imu_params.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  wm(2) = omega_inI(2) + true_bias_gyro(2) +
          params->imu_params.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  am(0) = accel_inI(0) + true_bias_accel(0) +
          params->imu_params.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
  am(1) = accel_inI(1) + true_bias_accel(1) +
          params->imu_params.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
  am(2) = accel_inI(2) + true_bias_accel(2) +
          params->imu_params.sigma_a / std::sqrt(dt) * w(gen_meas_imu);

  // Return success
  return true;
}

} // namespace msckf