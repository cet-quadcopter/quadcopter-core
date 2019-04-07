#include "falcon/control/velocity_control.h"

#include <iostream>

#include <math.h>
#include "falcon/math/quaternion.h"
#include "falcon/math/common.h"
#include "Eigen/Dense"


using namespace Eigen;
using namespace falcon::state;
using namespace falcon::math;
using namespace falcon::control;


VelocityControl::VelocityControl(VelocityControlParams params, double t0)
: params_(params), tm1_(t0), 
  pid_force_(params.force_kp, params.force_ki, params.force_kd),
  pid_torque_(params.torque_kp, params.torque_ki, params.torque_kd) {
  Matrix4f a;
  a << -params.kT           , -params.kT           , -params.kT           , -params.kT           ,
        0                   , -params.d * params.kT,  0                   ,  params.d * params.kT,
        params.d * params.kT,  0                   , -params.d * params.kT,  0                   ,
        params.kTau         , -params.kTau         ,  params.kTau         , -params.kTau         ;
  
  a_inv_ = a.inverse();
}

Vector4f VelocityControl::GetControlSignal(const StateManager& state, double t) {
  float dt = t - tm1_;

  const Vector3f& v_n = state.GetLinearVelocity();
  const Vector3f& g_n = state.GetGravity();
  const Vector4f& q_b_n = state.GetAttitude();

  Vector3f linear_velocity_error = v_n_desired_.head<3>() - v_n;

  Vector3f a_n = pid_force_.GetSignal(linear_velocity_error, dt);
  Vector3f f_n = params_.m * a_n - params_.m * g_n;

  Vector3f f_b = Vector3f(0, 0, sgn(f_n(2)) * f_n.norm());

  Vector4f q_error = QuaternionCalcError(q_b_n, f_n, f_b);
  Vector3f tau = pid_torque_.GetSignal(q_error.tail<3>(), dt);

  Vector4f w = Vector4f(f_b(2), tau(0), tau(1), tau(2));
  Vector4f omega = (a_inv_ * w).array().sqrt();

  tm1_ = t;

  return omega;
}

void VelocityControl::SetVelocity(const Vector4f& v) {
  v_n_desired_ = v;
}