#pragma once

#include "Eigen/Dense"
#include "falcon/math/calculus.h"


namespace falcon {
namespace control {


template <typename T, size_t N, size_t INTEGRATION_STEPS>
class PIDControl {
  private:
  const Eigen::Vector<T, N> kp_;
  const Eigen::Vector<T, N> ki_;
  const Eigen::Vector<T, N> kd_;

  math::Integrator<T, N, INTEGRATION_STEPS> integrator_;
  math::Differentiator<T, N> differentiator_;

  public:
  PIDControl(Eigen::Vector<T, N> kp, Eigen::Vector<T, N> ki, Eigen::Vector<T, N> kd)
  : kp_(kp), ki_(ki), kd_(kd), integrator_(Vector<T, N>::Zero()), differentiator_(Vector<T, N>::Zero()) {}

  Eigen::Vector<T, N> GetSignal(Eigen::Vector<T, N> error, float dt) {
    return kp_ * error + ki_ * integrator_.Update(error, dt) + kd_ * differentiator_.Update(error, dt);
  }
};


}
}