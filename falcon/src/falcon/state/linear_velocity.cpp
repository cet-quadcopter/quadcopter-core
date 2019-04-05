#include "falcon/state/linear_velocity.h"

#include "falcon/math/quaternion.h"

using namespace falcon::state;
using namespace falcon::math;
using namespace Eigen;


const Vector3f kInitialState = Vector3f::Zero();
const Matrix3f kInitialProcessCovariance = Matrix3f::Identity() * 10;
const Matrix3f kMeasurementTransform = Matrix3f::Identity();
const Matrix3f kProcessCovarianceTransform = Matrix3f::Identity();

const Matrix3f kA = Matrix3f::Identity();
const Vector3f kW = Vector3f::Zero();
const Vector3f kZ = Vector3f::Zero();


LinearVelocitySensor::LinearVelocitySensor(LinearVelocitySensorParams params)
: filter_(kInitialState, kInitialProcessCovariance, kMeasurementTransform, kProcessCovarianceTransform) {
  params_ = params;
}

void LinearVelocitySensor::PostControlInput(const Vector3f& a_b, const Vector4f& q_b_n, float dt) {
  Vector3f a_n = QuaternionRotate(q_b_n, a_b);
  Matrix3f b = DiagonalMatrix<float, 3>(dt, dt, dt);
  Matrix3f w = params_.covariance_accelerometer * dt;
  filter_.Predict(kA, a_n, b, kW, w);
}

void LinearVelocitySensor::PostMeasurementInput(const Vector3f& v_n) {
  filter_.Update(v_n, kZ, params_.covariance_gps_velocity);
}

const Vector3f& LinearVelocitySensor::GetLinearVelocity() {
  return filter_.GetState();
}