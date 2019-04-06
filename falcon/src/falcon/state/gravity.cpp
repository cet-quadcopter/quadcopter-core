#include "falcon/state/gravity.h"


using namespace falcon::state;
using namespace falcon::math;
using namespace Eigen;

const Vector3f kInitialState = Vector3f(0, 0, 9.8);

GravitySensor::GravitySensor(GravitySensorParams params)
: params_(params), filter_(params.alpha, kInitialState), gravity_diff_(kInitialState) {}

void GravitySensor::PostInput(const Vector3f& a_b, float dt) {
  auto diff = gravity_diff_.Update(a_b, dt).norm();

  if (diff < params_.threshold) {
    filter_.Update(Vector3f(0, 0, a_b.norm()));
  }
}

const Vector3f& GravitySensor::GetGravity() const {
  return filter_.GetState();
}