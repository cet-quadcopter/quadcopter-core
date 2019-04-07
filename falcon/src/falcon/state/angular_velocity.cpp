#include "falcon/state/angular_velocity.h"

#include "falcon/math/quaternion.h"

using namespace falcon::state;
using namespace falcon::math;
using namespace Eigen;


const Vector3f kAngularVelocityInitialState = Vector3f::Zero();
const Vector3f kAngularAccelerationInitialState = Vector3f::Zero();


AngularVelocitySensor::AngularVelocitySensor(AngularVelocitySensorParams params) 
: gyro_diff_(kAngularAccelerationInitialState), filter_(params.alpha, kAngularVelocityInitialState) {
  params_ = params;
}


void AngularVelocitySensor::PostInput(const Vector3f& g_b, const Vector4f& q_b_n, float dt) {
  Vector3f omega_n = QuaternionRotate(q_b_n, g_b);
  Vector3f alpha_n = gyro_diff_.Update(GetAngularVelocity(), dt);
  Vector3f omega_p = GetAngularVelocity() + alpha_n * dt;
  filter_.Update(omega_p, omega_n);
}


const Vector3f& AngularVelocitySensor::GetAngularVelocity() const {
  return filter_.GetState();
}