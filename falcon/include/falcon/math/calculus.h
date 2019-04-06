#pragma once

#include <algorithm>

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


template <typename T, size_t N, size_t LIMIT>
class Integrator {
  private:
  Eigen::Vector<T, N> x_tm1_;
  Eigen::Vector<T, N> integral_;
  Eigen::Vector<T, N> area_[LIMIT];
  size_t pointer_;

  public:
  Integrator(Eigen::Vector<T, N> initial_state): x_tm1_(initial_state), pointer_(0) {
    std::fill_n(area_, LIMIT, initial_state);
    integral_ = initial_state * LIMIT;
  }

  const Eigen::Vector<T, N>& Update(const Eigen::Vector<T, N>& value, float dt) {
    Eigen::Vector<T, N> trapz_area = (dt/2.0) * (value + x_tm1_);
    integral_ = integral_ - area_[pointer_] + trapz_area;

    area_[pointer_] = trapz_area;
    x_tm1_ = value;
    pointer_ = pointer_ < LIMIT - 1 ? pointer_ + 1 : 0;
    return integral_;
  }
};

}
}