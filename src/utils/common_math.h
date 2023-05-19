#pragma once
#include <Eigen/Dense>
namespace msckf {

/**
 * @brief Skew-symmetric matrix from a given 3x1 vector
 *
 * This is based on equation 6 in [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf): \f{align*}{
 *  \lfloor\mathbf{v}\times\rfloor =
 *  \begin{bmatrix}
 *  0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0
 *  \end{bmatrix}
 * @f}
 *
 * @param[in] w 3x1 vector to be made a skew-symmetric
 * @return 3x3 skew-symmetric matrix
 */
inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}

/**
 * @brief Returns vector portion of skew-symmetric
 *
 * See skew_x() for details.
 *
 * @param[in] w_x skew-symmetric matrix
 * @return 3x1 vector portion of skew
 */
inline Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &w_x) {
  Eigen::Matrix<double, 3, 1> w;
  w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
  return w;
}

/**
 * @brief SO(3) matrix exponential
 *
 * SO(3) matrix exponential mapping from the vector to SO(3) lie group.
 * This formula ends up being the [Rodrigues
 * formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula). This definition was taken
 * from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 15.
 * http://ethaneade.com/lie.pdf
 *
 * \f{align*}{
 * \exp\colon\mathfrak{so}(3)&\to SO(3) \\
 * \exp(\mathbf{v}) &=
 * \mathbf{I}
 * +\frac{\sin{\theta}}{\theta}\lfloor\mathbf{v}\times\rfloor
 * +\frac{1-\cos{\theta}}{\theta^2}\lfloor\mathbf{v}\times\rfloor^2 \\
 * \mathrm{where}&\quad \theta^2 = \mathbf{v}^\top\mathbf{v}
 * @f}
 *
 * @param[in] w 3x1 vector in R(3) we will take the exponential of
 * @return SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> exp_so3(const Eigen::Matrix<double, 3, 1> &w) {
  // get theta
  Eigen::Matrix<double, 3, 3> w_x = skew_x(w);
  double theta = w.norm();
  // Handle small angle values
  double A, B;
  if (theta < 1e-7) {
    A = 1;
    B = 0.5;
  } else {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
  }
  // compute so(3) rotation
  Eigen::Matrix<double, 3, 3> R;
  if (theta == 0) {
    R = Eigen::MatrixXd::Identity(3, 3);
  } else {
    R = Eigen::MatrixXd::Identity(3, 3) + A * w_x + B * w_x * w_x;
  }
  return R;
}

/**
 * @brief SO(3) matrix logarithm
 *
 * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation
 * 17 & 18. http://ethaneade.com/lie.pdf \f{align*}{
 * \theta &= \textrm{arccos}(0.5(\textrm{trace}(\mathbf{R})-1)) \\
 * \lfloor\mathbf{v}\times\rfloor &= \frac{\theta}{2\sin{\theta}}(\mathbf{R}-\mathbf{R}^\top)
 * @f}
 *
 * This function is based on the GTSAM one as the original implementation was a bit unstable.
 * See the following:
 * - https://github.com/borglab/gtsam/
 * - https://github.com/borglab/gtsam/issues/746
 * - https://github.com/borglab/gtsam/pull/780
 *
 * @param[in] R 3x3 SO(3) rotation matrix
 * @return 3x1 in the R(3) space [omegax, omegay, omegaz]
 */
inline Eigen::Matrix<double, 3, 1> log_so3(const Eigen::Matrix<double, 3, 3> &R) {

  // note switch to base 1
  double R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  double R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  double R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const double tr = R.trace();
  Eigen::Vector3d omega;

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (tr + 1.0 < 1e-10) {
    if (std::abs(R33 + 1.0) > 1e-5)
      omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
    else if (std::abs(R22 + 1.0) > 1e-5)
      omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
    else
      // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
      omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // always negative
    if (tr_3 < -1e-7) {
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0;
    }
    omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
  }

  return omega;
}

/**
 * @brief SE(3) matrix exponential function
 *
 * Equation is from Ethan Eade's reference: http://ethaneade.com/lie.pdf
 * \f{align*}{
 * \exp([\boldsymbol\omega,\mathbf u])&=\begin{bmatrix} \mathbf R & \mathbf V \mathbf u \\ \mathbf 0
 * & 1 \end{bmatrix} \\[1em]
 * \mathbf R &= \mathbf I + A \lfloor \boldsymbol\omega \times\rfloor + B \lfloor \boldsymbol\omega
 * \times\rfloor^2 \\ \mathbf V &= \mathbf I + B \lfloor \boldsymbol\omega \times\rfloor + C \lfloor
 * \boldsymbol\omega \times\rfloor^2 \f} where we have the following definitions \f{align*}{
 * \theta &= \sqrt{\boldsymbol\omega^\top\boldsymbol\omega} \\
 * A &= \sin\theta/\theta \\
 * B &= (1-\cos\theta)/\theta^2 \\
 * C &= (1-A)/\theta^2
 * \f}
 *
 * @param vec 6x1 in the R(6) space [omega, u]
 * @return 4x4 SE(3) matrix
 */
inline Eigen::Matrix4d exp_se3(Eigen::Matrix<double, 6, 1> vec) {

  // Precompute our values
  Eigen::Vector3d w = vec.head(3);
  Eigen::Vector3d u = vec.tail(3);
  double theta = sqrt(w.dot(w));
  Eigen::Matrix3d wskew;
  wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // Handle small angle values
  double A, B, C;
  if (theta < 1e-7) {
    A = 1;
    B = 0.5;
    C = 1.0 / 6.0;
  } else {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
    C = (1 - A) / (theta * theta);
  }

  // Matrices we need V and Identity
  Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = I_33 + B * wskew + C * wskew * wskew;

  // Get the final matrix to return
  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.block(0, 0, 3, 3) = I_33 + A * wskew + B * wskew * wskew;
  mat.block(0, 3, 3, 1) = V * u;
  mat(3, 3) = 1;
  return mat;
}

/**
 * @brief SE(3) matrix logarithm
 *
 * Equation is from Ethan Eade's reference: http://ethaneade.com/lie.pdf
 * \f{align*}{
 * \boldsymbol\omega &=\mathrm{skew\_offdiags}\Big(\frac{\theta}{2\sin\theta}(\mathbf R - \mathbf
 * R^\top)\Big) \\ \mathbf u &= \mathbf V^{-1}\mathbf t \f} where we have the following definitions
 * \f{align*}{
 * \theta &= \mathrm{arccos}((\mathrm{tr}(\mathbf R)-1)/2) \\
 * \mathbf V^{-1} &= \mathbf I - \frac{1}{2} \lfloor \boldsymbol\omega \times\rfloor +
 * \frac{1}{\theta^2}\Big(1-\frac{A}{2B}\Big)\lfloor \boldsymbol\omega \times\rfloor^2 \f}
 *
 * This function is based on the GTSAM one as the original implementation was a bit unstable.
 * See the following:
 * - https://github.com/borglab/gtsam/
 * - https://github.com/borglab/gtsam/issues/746
 * - https://github.com/borglab/gtsam/pull/780
 *
 * @param mat 4x4 SE(3) matrix
 * @return 6x1 in the R(6) space [omega, u]
 */
inline Eigen::Matrix<double, 6, 1> log_se3(Eigen::Matrix4d mat) {
  Eigen::Vector3d w = log_so3(mat.block<3, 3>(0, 0));
  Eigen::Vector3d T = mat.block<3, 1>(0, 3);
  const double t = w.norm();
  if (t < 1e-10) {
    Eigen::Matrix<double, 6, 1> log;
    log << w, T;
    return log;
  } else {
    Eigen::Matrix3d W = skew_x(w / t);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in T to avoid matrix math
    double Tan = tan(0.5 * t);
    Eigen::Vector3d WT = W * T;
    Eigen::Vector3d u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    Eigen::Matrix<double, 6, 1> log;
    log << w, u;
    return log;
  }
}

/**
 * @brief Hat operator for R^6 -> Lie Algebra se(3)
 *
 * \f{align*}{
 * \boldsymbol\Omega^{\wedge} = \begin{bmatrix} \lfloor \boldsymbol\omega \times\rfloor & \mathbf u
 * \\ \mathbf 0 & 0 \end{bmatrix} \f}
 *
 * @param vec 6x1 in the R(6) space [omega, u]
 * @return Lie algebra se(3) 4x4 matrix
 */
inline Eigen::Matrix4d hat_se3(const Eigen::Matrix<double, 6, 1> &vec) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.block(0, 0, 3, 3) = skew_x(vec.head(3));
  mat.block(0, 3, 3, 1) = vec.tail(3);
  return mat;
}

/**
 * @brief SE(3) matrix analytical inverse
 *
 * It seems that using the .inverse() function is not a good way.
 * This should be used in all cases we need the inverse instead of numerical inverse.
 * https://github.com/rpng/open_vins/issues/12
 * \f{align*}{
 * \mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^\top & -\mathbf{R}^\top\mathbf{p} \\ \mathbf{0} & 1
 * \end{bmatrix} \f}
 *
 * @param[in] T SE(3) matrix
 * @return inversed SE(3) matrix
 */
inline Eigen::Matrix4d Inv_se3(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
  T_inv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  T_inv.block(0, 3, 3, 1) = -T_inv.block(0, 0, 3, 3) * T.block(0, 3, 3, 1);
  return T_inv;
}

} // namespace msckf
