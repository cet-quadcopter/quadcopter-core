#pragma once

#include <memory>

#include "Eigen/Dense"
#include "pid.h"
#include "falcon/state/manager.h"

namespace falcon {
namespace control {

struct VelocityControlParams {
  Eigen::Vector3f force_kp;
  Eigen::Vector3f force_ki;
  Eigen::Vector3f force_kd;
  Eigen::Vector3f torque_kp;
  Eigen::Vector3f torque_ki;
  Eigen::Vector3f torque_kd;

  float m;
  float kT;
  float kTau;
  float d;
};


class VelocityControl {
  private:
  const VelocityControlParams params_;
  Eigen::Matrix4f a_inv_;

  Eigen::Vector4f v_n_desired_;

  PIDControl<float, 3, 1000> pid_force_;
  PIDControl<float, 3, 1000> pid_torque_;

  double tm1_;

  public:
  VelocityControl(VelocityControlParams params, double t0);
  Eigen::Vector4f GetControlSignal(const state::StateManager& state_, double t);
  void SetVelocity(const Eigen::Vector4f& velocity);
};

}
}