#include "imu_params.h"
#include "sim_params.h"
namespace msckf {

struct Params {
  ImuParams imu_params;
  SimParams sim_params;
  double gravity_mag = 9.81;
};
} // namespace msckf