#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <random>

namespace msckf {
class Params;
class BsplineSE3;

/**
 * @brief Master simulator class that generated visual-inertial measurements
 *
 * Given a trajectory this will generate a SE(3) @ref ov_core::BsplineSE3 for that trajectory.
 * This allows us to get the inertial measurement information at each time step during this
 * trajectory. After creating the bspline we will generate an environmental feature map which will
 * be used as our feature measurements. This map will be projected into the frame at each time step
 * to get our "raw" uv measurements. We inject bias and white noises into our inertial readings
 * while adding our white noise to the uv measurements also. The user should specify the sensor
 * rates that they desire along with the seeds of the random number generators.
 *
 */
class Simulator {

public:
  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ parameters. Should have already been loaded from cmd.
   */
  Simulator(const Params &params_);

  /**
   * @brief Returns if we are actively simulating
   * @return True if we still have simulation data
   */
  bool ok() { return is_running; }

  /**
   * @brief Gets the timestamp we have simulated up too
   * @return Timestamp
   */
  double current_timestamp() { return timestamp; }

  /**
   * @brief Get the simulation state at a specified time step
   * @param desired_time Timestamp we want to get the state at
   * @param imu_state State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   * @return True if we have a state
   */
  bool get_state(double desired_time, Eigen::Matrix<double, 17, 1> &imu_state);

  /**
   * @brief Gets the next inertial reading if we have one.
   * @param time_imu Time that this measurement occured at
   * @param wm Angular velocity measurement in the inertial frame
   * @param am Linear velocity in the inertial frame
   * @return True if we have a measurement
   */
  bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

protected:
  //===================================================================
  // Configuration variables
  //===================================================================

  /// True vio manager params (a copy of the parsed ones)
  std::shared_ptr<Params> params;

  //===================================================================
  // State related variables
  //===================================================================

  /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
  std::vector<Eigen::VectorXd> traj_data;

  /// Our b-spline trajectory
  std::shared_ptr<BsplineSE3> spline;

  /// Mersenne twister PRNG for measurements (IMU)
  std::mt19937 gen_meas_imu;

  /// Mersenne twister PRNG for state initialization
  std::mt19937 gen_state_init;

  /// Mersenne twister PRNG for state perturbations
  std::mt19937 gen_state_perturb;

  /// If our simulation is running
  bool is_running;

  //===================================================================
  // Simulation specific variables
  //===================================================================

  /// Current timestamp of the system
  double timestamp;

  /// Last time we had an IMU reading
  double timestamp_last_imu;

  /// Our running acceleration bias
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

  /// Our running gyroscope bias
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // Our history of true biases
  bool has_skipped_first_bias = false;
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;
};

} // namespace msckf
