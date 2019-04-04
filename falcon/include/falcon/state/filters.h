#pragma once

#include <Eigen/Dense>
#include "../utils.h"


namespace falcon {
namespace state {

using Eigen::Matrix;
using Eigen::Vector;


template <typename T, size_t N, size_t U, size_t Y>
class KalmanFilter {
  protected:
  Vector<T, N> x_tm1_;
  Matrix<T, N, N> p_tm1_;

  Matrix<T, N, Y> c_;
  Matrix<T, N, N> h_;

  public:
  KalmanFilter(Vector<T, N> x_0, Matrix<T, N, N> p_0, Matrix<T, N, Y> c, Matrix<T, N, N> h) {
    x_tm1_ = x_0;
    p_tm1_ = p_0;

    c_ = c;
    h_ = h;
  }

  void Update(
    const Vector<T, Y>& m_t, const Vector<T, N>& z_t, const Matrix<T, N, N> r_t
  ) {
    auto y_t = c_ * m_t + z_t;
    auto k = p_tm1_ * (p_tm1_ + r_t).inverse();

    x_tm1_ = x_tm1_ + k * (y_t - x_tm1_);
    p_tm1_ = p_tm1_ - k * p_tm1_;
  }

  void Predict(
    const Matrix<T, N, N>& a, const Vector<T, U>& u_t, const Matrix<T, N, U> b,
    const Vector<T, N>& w_t, const Matrix<T, N, N>& q_t
  ) {
    x_tm1_ = a * x_tm1_ + b * u_t + w_t;
    p_tm1_ = a * p_tm1_ * a.transpose() + q_t;
  }

  const Vector<T, N>& GetState() {
    return x_tm1_;
  }
};

template <typename T, size_t N>
class ComplementaryFilter {
  protected:
  Vector<T, N> alpha_;
  Vector<T, N> alpha_inv_;
  Vector<T, N> x_tm1_;

  public:
  ComplementaryFilter(Vector<T, N> alpha, Vector<T, N> initial_state)
  : alpha_(alpha), alpha_inv_(Vector<T, N>::Ones() - alpha) {
    x_tm1_ = initial_state;
  }

  void Update(const Vector<T, N>& prediction, const Vector<T, N>& measurement) {
    x_tm1_ = alpha_inv_.cwiseProduct(prediction) + alpha_.cwiseProduct(measurement);
  }

  const Vector<T, N>& GetState() {
    return x_tm1_;
  }
};

};
};