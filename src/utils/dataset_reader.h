#pragma once

#include "log.h"
#include <Eigen/Eigen>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
using namespace std;

namespace msckf {

class DatasetReader {

public:
  /**
   * @brief Load a ASL format ground truth file
   * @param path Path to the CSV file of ground truth data
   * @param gt_states Will be filled with ground truth states
   *
   * Here we will try to load a ground truth file that is in the ASL/EUROC_MAV format.
   * If we can't open the file, or it is in the wrong format we will error and exit the program.
   * See get_gt_state() for a way to get the ground truth state at a given time step
   */
  static void load_gt_file(std::string path,
                           std::map<double, Eigen::Matrix<double, 17, 1>> &gt_states) {

    // Clear any old data
    gt_states.clear();

    // Open the file
    std::ifstream file;
    std::string line;
    file.open(path);

    // Check that it was successful
    if (!file) {
      AERROR << "ERROR: Unable to open groundtruth file... " << path;
      std::exit(EXIT_FAILURE);
    }

    // Skip the first line as it is just the header
    std::getline(file, line);

    // Loop through each line in the file
    while (std::getline(file, line)) {
      // Loop variables
      int i = 0;
      std::istringstream s(line);
      std::string field;
      Eigen::Matrix<double, 17, 1> temp = Eigen::Matrix<double, 17, 1>::Zero();
      // Loop through this line
      while (getline(s, field, ',')) {
        // Ensure we are in the range
        if (i > 16) {
          AERROR << "ERROR: Invalid groundtruth line, too long!" << line;
          std::exit(EXIT_FAILURE);
        }
        // Save our ground truth state value
        temp(i, 0) = std::atof(field.c_str());
        i++;
      }
      // Append to our ground truth map
      gt_states.insert({1e-9 * temp(0, 0), temp});
    }
    file.close();
  }

  /**
   * @brief Gets the 17x1 groundtruth state at a given timestep
   * @param timestep timestep we want to get the groundtruth for
   * @param imu_state groundtruth state [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   * @param gt_states Should be loaded with groundtruth states, see load_gt_file() for details
   * @return true if we found the state, false otherwise
   */
  static bool get_gt_state(double timestep, Eigen::Matrix<double, 17, 1> &imu_state,
                           std::map<double, Eigen::Matrix<double, 17, 1>> &gt_states) {

    // Check that we even have groundtruth loaded
    if (gt_states.empty()) {
      AERROR << "Groundtruth data loaded is empty, make sure you call load before asking for "
                "a state.";
      return false;
    }

    // Loop through gt states and find the closest time stamp
    double closest_time = INFINITY;
    auto it0 = gt_states.begin();
    while (it0 != gt_states.end()) {
      if (std::abs(it0->first - timestep) < std::abs(closest_time - timestep)) {
        closest_time = it0->first;
      }
      it0++;
    }

    // If close to this timestamp, then use it
    if (std::abs(closest_time - timestep) < 0.10) {
      // ADEBUG << "init DT = " << std::abs(closest_time - timestep);
      // ADEBUG << "timestamp = " << closest_time;
      timestep = closest_time;
    }

    // Check that we have the timestamp in our GT file
    if (gt_states.find(timestep) == gt_states.end()) {
      // AWARN << "Unable to find " << timestep << "  in GT file, wrong GT file loaded ? ? ? ";
      return false;
    }

    // Get the GT state vector
    Eigen::Matrix<double, 17, 1> state = gt_states[timestep];

    // Our "fixed" state vector from the ETH GT format [q,p,v,bg,ba]
    imu_state(0, 0) = timestep;    // time
    imu_state(1, 0) = state(5, 0); // quat
    imu_state(2, 0) = state(6, 0);
    imu_state(3, 0) = state(7, 0);
    imu_state(4, 0) = state(4, 0);
    imu_state(5, 0) = state(1, 0); // pos
    imu_state(6, 0) = state(2, 0);
    imu_state(7, 0) = state(3, 0);
    imu_state(8, 0) = state(8, 0); // vel
    imu_state(9, 0) = state(9, 0);
    imu_state(10, 0) = state(10, 0);
    imu_state(11, 0) = state(11, 0); // bg
    imu_state(12, 0) = state(12, 0);
    imu_state(13, 0) = state(13, 0);
    imu_state(14, 0) = state(14, 0); // ba
    imu_state(15, 0) = state(15, 0);
    imu_state(16, 0) = state(16, 0);

    // Success!
    return true;
  }

  /**
   * @brief This will load the trajectory into memory (space separated)
   * @param path Path to the trajectory file that we want to read in.
   * @param traj_data Will be filled with groundtruth states (timestamp(s), q_GtoI, p_IinG)
   */
  static void load_simulated_trajectory(std::string path, std::vector<Eigen::VectorXd> &traj_data) {

    // Try to open our groundtruth file
    std::ifstream file;
    file.open(path);
    if (!file) {
      AERROR << "ERROR: Unable to open simulation trajectory file." << path;
      std::exit(EXIT_FAILURE);
    }

    // Debug print
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    ADEBUG << "loaded trajectory " << base_filename;

    // Loop through each line of this file
    std::string current_line;
    while (std::getline(file, current_line)) {

      // Skip if we start with a comment
      if (!current_line.find("#"))
        continue;

      // Loop variables
      int i = 0;
      std::istringstream s(current_line);
      std::string field;
      Eigen::Matrix<double, 8, 1> data;

      // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
      while (std::getline(s, field, ' ')) {
        // Skip if empty
        if (field.empty() || i >= data.rows())
          continue;
        // save the data to our vector
        data(i) = std::atof(field.c_str());

        // because open_vins simulation data quaternion form is jpl, trans it to hamilton
        if (i >= 4 && i <= 6) {
          data(i) = -data(i);
        }

        i++;
      }

      // Only a valid line if we have all the parameters
      if (i > 7) {
        traj_data.push_back(data);
        ADEBUG << std::setprecision(15) << data.transpose();
      }
    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (traj_data.empty()) {
      AERROR << "ERROR: Could not parse any data from the file!!" << path;
      std::exit(EXIT_FAILURE);
    }
  }

private:
  /**
   * All function in this class should be static.
   * Thus an instance of this class cannot be created.
   */
  DatasetReader() {}
};

} // namespace msckf
