#include "falcon/state/attitude.h"

#include <math.h>
#include <cmath>
#include "falcon/math/quaternion.h"

using namespace falcon::state;
using namespace falcon::math;
using namespace Eigen;

const Vector4f kInitialState = Vector4f(1, 0, 0, 0);
const Matrix4f kInitialProcessCovariance = Matrix4f::Identity() * 10;
const Matrix4f kMeasurementTransform = Matrix4f::Identity();
const Matrix4f kProcessCovarianceTransform = Matrix4f::Identity();

const Matrix4f kPreviousStateTransform = Matrix4f::Identity();
const Vector4f kPredictedStateNoise = Vector4f::Zero();
const Vector4f kMeasuredStateNoise = Vector4f::Zero();

const Vector3f kGravityCap = Vector3f(0, 0, 1);
constexpr float kMagneticInclination = 7.65 * M_PI / 180.0;
const Vector3f kNorthCap = Vector3f(std::cos(kMagneticInclination), std::sin(kMagneticInclination), 0);


AttitudeFilter::AttitudeFilter(
  Vector<float, 4> x_0, Matrix<float, 4, 4> p_0, Matrix<float, 4, 4> c, Matrix<float, 4, 4> h)
  : KalmanFilter(x_0, p_0, c, h) {}

void AttitudeFilter::Predict(
  const Matrix<float, 4, 4>& a, const Vector<float, 3>& u_t, const Matrix<float, 4, 3> b,
  const Vector<float, 4>& w_t, const Matrix<float, 4, 4>& q_t) {
  
  KalmanFilter::Predict(a, u_t, b, w_t, q_t);
  x_tm1_.normalize();
}



AttitudeSensor::AttitudeSensor(AttitudeSensorParams params): filter_(kInitialState, kInitialProcessCovariance, kMeasurementTransform, kProcessCovarianceTransform) {
  params_ = params;
}

void AttitudeSensor::PostControlInput(const Vector3f& gyro, float dt) {
  Vector4f q = filter_.GetState();

  Matrix<float, 4, 3> b;
  b << -q(0), -q(2), -q(3),
        q(0), -q(3),  q(2),
        q(3),  q(0), -q(1),
       -q(2),  q(1),  q(0);
  b = b * dt / 2;

  Matrix<float, 4, 3> g;
  g << q(1),  q(2),  q(3),
      -q(0),  q(3), -q(2),
      -q(3), -q(0),  q(1),
       q(2), -q(1), -q(0);
  
  auto process_covariance = g * params_.covariance_gyro * g.transpose() * (std::pow(dt, 2) / 4);
  
  filter_.Predict(kPreviousStateTransform, gyro, b, kPredictedStateNoise, process_covariance);
}

void AttitudeSensor::PostMeasurementInput(const Vector3f& a_b, const Vector3f& m_b) {
  Vector4f q = GetAttitude();
  Vector4f q_g = QuaternionCalcRotation(q, kGravityCap, a_b.normalized(), params_.accelerometer_gain);
  
  Vector3f d_b = QuaternionRotate(q_g, kGravityCap);
  Vector3f e_b = d_b.cross(m_b.normalized());
  Vector3f n_b = e_b.cross(d_b);

  Vector4f q_m = QuaternionCalcRotation(q_g, kNorthCap, n_b, params_.magnetometer_gain);

  Matrix4f r = Matrix4f::Identity() * 0.0001;

  filter_.Update(q_m, kMeasuredStateNoise, r);
}

const Vector4f& AttitudeSensor::GetAttitude() const {
  return filter_.GetState();
}
