#include "falcon/state/velocity.h"

#include "falcon/math/quaternion.h"

using namespace falcon::state;
using namespace falcon::math;
using namespace Eigen;


const Vector3f kLinearVelocityInitialState = Vector3f::Zero();
const Vector3f kAngularVelocityInitialState = Vector3f::Zero();
const Vector3f kAngularAccelerationInitialState = Vector3f::Zero();


VelocitySensor::VelocitySensor(VelocitySensorParams params) 
: gyro_diff_(kAngularAccelerationInitialState),
  linear_velocity_filter_(params.alpha_linear_velocity, kLinearVelocityInitialState),
  angular_velocity_filter_(params.alpha_angular_velocity, kAngularVelocityInitialState) {
  params_ = params;
}


void VelocitySensor::PostMeasurementInput(
  const Vector3f& a_b, const Vector3f& g_b, const Vector3f& v_n, const Vector4f& q_b_n, float dt) {
  auto a_n = QuaternionRotate(q_b_n, a_b);
  auto v_p = GetLinearVelocity() + a_n * dt;
  linear_velocity_filter_.Update(v_p, v_n);

  auto omega_n = QuaternionRotate(q_b_n, g_b);
  auto alpha_n = gyro_diff_.Update(GetAngularVelocity(), dt);
  auto omega_p = GetAngularVelocity() + alpha_n * dt;
  angular_velocity_filter_.Update(omega_p, omega_n);
}


const Vector3f& VelocitySensor::GetAngularVelocity() {
  return angular_velocity_filter_.GetState();
}


const Vector3f& VelocitySensor::GetLinearVelocity() {
  return linear_velocity_filter_.GetState();
}