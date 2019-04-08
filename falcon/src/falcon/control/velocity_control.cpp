#include "falcon/control/velocity_control.h"

#include <iostream>
#include <cmath>

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

  float kT = params.kT;
  float kTau = params.kTau;
  float dkT = params.d * kT / std::sqrt(2);

  Matrix4f a;
  a << -kT  , -kT , -kT  , -kT  ,
        dkT , -dkT,  dkT , -dkT ,
       -dkT ,  dkT,  dkT , -dkT ,
        kTau, kTau, -kTau, -kTau;
  
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
  float fx_max = abs(f_n(2) * params_.fx_max_factor);
  float fy_max = abs(f_n(2) * params_.fy_max_factor);
  f_n(0) = fmax(-fx_max, fmin(fx_max, f_n(0)));
  f_n(1) = fmax(-fy_max, fmin(fy_max, f_n(1)));

  Vector3f f_b = Vector3f(0, 0, sgn(f_n(2)) * f_n.norm());

  Vector4f q_error = QuaternionCalcError(QuaternionConjugate(q_b_n), f_b, f_n);

  std::cout << "f_n" << std::endl;
  std::cout << f_n << std::endl;

  std::cout << "f_b" << std::endl;
  std::cout << f_b << std::endl;

  std::cout << "q_error" << std::endl;
  std::cout << q_error << std::endl;

  Vector3f tau = pid_torque_.GetSignal(q_error.tail<3>(), dt);

  std::cout << "tau" << std::endl;
  std::cout << tau << std::endl;

  Vector4f w = Vector4f(f_b(2), 0, -tau(1), 0);
  Vector4f omega = (a_inv_ * w).array().sqrt();

  if (omega.maxCoeff() > 1) {
    omega /= omega.maxCoeff();
  }

  std::cout << "omega" << std::endl;
  std::cout << omega << std::endl;

  std::cout << "v_n error" << std::endl;
  std::cout << v_n_desired_.head<3>() - v_n << std::endl;

  tm1_ = t;

  return omega;
}

void VelocityControl::SetVelocity(const Vector4f& v) {
  v_n_desired_ = v;
}