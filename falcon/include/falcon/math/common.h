#pragma once

#include <Eigen/Dense>
#include "falcon/utils.h"


namespace falcon {
namespace math {

template <typename T, size_t N>
class Accumulator {
  private:
  Eigen::Vector<T, N> value_;
  size_t count_;

  public:
  Accumulator(): value_(Eigen::Vector<T, N>::Zero()), count_(0) {}

  void Post(const Eigen::Vector<T, N>& value) {
    value_ += value;
    count_++;
  }

  bool HasData() {
    return count_ > 0;
  }

  Eigen::Vector<T, N> Get() {
    if (count_ < 1) {
      throw "Attempt to get data from accumulator with no data";
    }

    return value_ / count_;
  }

  Eigen::Vector<T, N> GetAndReset() {
    auto result = Get();
    Reset();
    return result;
  }

  void Reset() {
    value_ *= 0.0;
    count_ = 0;
  }
};

}
}