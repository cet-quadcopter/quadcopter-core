#pragma once

#include <iostream>
#include <chrono>
#include <utility>

#include <Eigen/Dense>

namespace Eigen {

template <typename T, size_t N>
using Vector = Eigen::Matrix<T, N, 1>;

}

namespace falcon {
namespace utils {

template<typename F, typename... Args>
void measureFunction(size_t count, F func, Args&&... args){
  std::chrono::nanoseconds duration(0);

  for (size_t i = 0; i < count; i++) {
    auto start = std::chrono::high_resolution_clock::now();
    func(std::forward<Args>(args)...);
    duration += std::chrono::high_resolution_clock::now() - start;
  }

  double ns = duration.count() / count;

  std::cout << "Duration ns : " <<  ns << std::endl;
  std::cout << "Duration us : " << ns / 1e3 << std::endl;
  std::cout << "Duration ms : " << ns / 1e6 << std::endl;
}

}
}