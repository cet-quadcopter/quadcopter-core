#pragma once

#include <cmath>

#include <Eigen/Dense>
#include "../utils.h"

using namespace Eigen;

namespace falcon {
namespace math {


template <typename T>
inline Vector<T, 4> QuaternionMultiply(const Vector<T, 4>& p, const Vector<T, 4>& q) {
  Vector<T, 4> result(
    p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
    p(0) * q(1) + p(1) * q(0) + p(2) * q(3) - p(3) * q(2),
    p(0) * q(2) - p(1) * q(3) + p(2) * q(0) + p(3) * q(1),
    p(0) * q(3) + p(1) * q(2) - p(2) * q(1) + p(3) * q(0)
  );

  return result;
}


template <typename T>
inline Vector<T, 4> QuaternionConjugate(const Vector<T, 4>& q) {
  return Vector<T, 4>(q(0), -q(1), -q(2), -q(3));
}


template <typename T>
inline Vector<T, 3> QuaternionRotate(const Vector<T, 4>& q, const Vector<T, 3>& v) {
  Vector<T, 4> result = QuaternionMultiply(
    QuaternionMultiply(q, Vector<T, 4>(0, v(0), v(1), v(2))), 
    QuaternionConjugate(q)
  );

  return Vector<T, 3>(result(1), result(2), result(3));
}


template <typename T>
inline Vector<T, 4> QuaternionCalcError(
  const Vector<T, 4>& q, const Vector<T, 3>& v_n, const Vector<T, 3>& v_b, float gain=1
) {
  Vector<T, 3> v_n_est = QuaternionRotate(q, v_b);
  float v_error = std::acos(fmax(-1, fmin(1, v_n_est.dot(v_n) / (v_n_est.norm() * v_n.norm()))));

  if (v_error == 0) {
    return q;
  }

  Vector<T, 3> v_rot_axis = v_n_est.cross(v_n).normalized() * std::sin(v_error * gain / 2);

  Vector<T, 4> q_error = Vector<T, 4>(
    std::cos(v_error * gain / 2), v_rot_axis(0), v_rot_axis(1), v_rot_axis(2));

  return q_error;
}


template <typename T>
inline Vector<T, 4> QuaternionCalcRotation(
  const Vector<T, 4>& q, const Vector<T, 3>& v_n, const Vector<T, 3>& v_b, float gain
) {
  return QuaternionMultiply(QuaternionCalcError(q, v_n, v_b, gain), q);
}

template <typename T>
inline Vector<T, 3> QuaternionToEuler123(const Vector<T, 4>& q) {
  T psi = std::atan2(2 * q(2) * q(3) + 2 * q(0) * q(1), q(3) * q(3) - q(2) * q(2) - q(1) * q(1) + q(0) * q(0));
  T theta = -std::asin(2 * q(1) * q(3) - 2 * q(0) * q(2));
  T gamma = std::atan2(2 * q(1) * q(2) + 2 * q(0) * q(3), q(1) * q(1) + q(0) * q(0) - q(3) * q(3) - q(2) * q(2));

  return Vector<T, 3>(psi, theta, gamma);
}

}
}