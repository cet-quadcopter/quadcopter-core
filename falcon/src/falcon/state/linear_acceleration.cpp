#include "falcon/state/linear_acceleration.h"

#include "falcon/math/quaternion.h"


using namespace falcon::state;
using namespace falcon::math;
using namespace Eigen;

const Vector3f kInitialState = Vector3f(0, 0, 0);

LinearAccelerationSensor::LinearAccelerationSensor()
: x_t_(kInitialState) {}

void LinearAccelerationSensor::PostInput(const Vector3f& a_b, const Vector3f& g_n, const Vector4f& q_b_n) {
  Vector3f a_n = QuaternionRotate(q_b_n, a_b);
  x_t_ = g_n - a_n;
}

const Vector3f& LinearAccelerationSensor::GetLinearAcceleration() const {
  return x_t_;
}