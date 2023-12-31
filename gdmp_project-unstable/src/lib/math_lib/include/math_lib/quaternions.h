#ifndef MATH_LIB_UNIT_QUATERNIONS_64_H
#define MATH_LIB_UNIT_QUATERNIONS_64_H

#include <Eigen/Dense>
#include <armadillo>
#include <math_lib/math.h>

namespace as64_
{

namespace math_
{

Eigen::Vector4d quatProd(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2);
arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);

Eigen::Vector4d quatInv(const Eigen::Vector4d &quat);
arma::vec quatInv(const arma::vec &quat);

Eigen::Vector4d quatExp(const Eigen::Vector3d &v_rot, double zero_tol=1e-16);
arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);

Eigen::Vector3d quatLog(const Eigen::Vector4d &quat, double zero_tol=1e-16);
arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16);

Eigen::Matrix4d quat2qmat(const Eigen::Vector4d &quat);
arma::mat quat2qmat(const arma::vec &quat);

// If quat1 and quat2 were positions, this would perform quat1-quat2.
// The result is the amount of rotation needed to go from quat2 to quat1, i.e. quatD*quat2 = quat1
// Equivalently, the result is quaternion corresponding to the angular velocity which takes us from quat2 to quat1 in unit time.
Eigen::Vector4d quatDiff(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2);
arma::vec quatDiff(const arma::vec &quat1, const arma::vec &quat2);


arma::vec rotVel_to_qLogDot(const arma::vec &rotVel, const arma::vec &logQ);

arma::vec qLogDot_to_rotVel(const arma::vec &logQ_dot, const arma::vec &Q);

arma::vec rotAccel_to_qLogDDot(const arma::vec &rotAccel, const arma::vec &rotVel, const arma::vec &Q);

arma::vec qLogDDot_to_rotAccel(const arma::vec &logQ_ddot, const arma::vec &rotVel, const arma::vec &Q);

arma::mat jacob_Q_qLog(const arma::vec &Q);

arma::mat jacob_qLog_Q(const arma::vec &Q);

arma::mat jacobDot_qLog_Q(const arma::vec &Q, const arma::vec &rotVel);

arma::mat jacobDot_Q_qLog(const arma::vec &Q, const arma::vec &rotVel);

} // namespace math_

} // namespace as64_

#endif // MATH_LIB_UNIT_QUATERNIONS_64_H
