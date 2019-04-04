#pragma once

#include <Eigen/Dense>


namespace falcon {
namespace math {

template <typename T, size_t N>
class Differentiator {
  private:
  Eigen::Vector<T, N> x_tm1_;

  public:
  Differentiator(Eigen::Vector<T, N> initial_state) {
    x_tm1_ = initial_state;
  }

  Eigen::Vector<T, N> Update(const Eigen::Vector<T, N>& value, float dt) {
    auto result = (value - x_tm1_) / dt;

    x_tm1_ = value;

    return result;
  }
};

}
}