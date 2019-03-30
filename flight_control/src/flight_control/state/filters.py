import time

import numpy as np
from ..utils import unimplemented


"""
Abstract kalman filter implementation
"""
class KalmanFilter(object):

  def __init__(self, process_error_covariance_transform, measurement_transform, 
    initial_state, initial_predicted_error_covariance):
    self.h = process_error_covariance_transform
    self.c = measurement_transform

    self.x_t_m1 = initial_state
    self.p_t_m1 = initial_predicted_error_covariance
    self.t_m1 = time.time()


  def update(self, measurement, t):
    assert t > self.t_m1

    y_t = self._measure_current_state(measurement, t)
    prediction = (self.x_t_m1, self.p_t_m1)
    x_t, p_t = self._combine_prediction_and_measurement(prediction, measurement, t)

    self.t_m1 = t
    self.x_t_m1 = x_t
    self.p_t_m1 = p_t

    return x_t


  def predict(self, control_input, t):
    assert t > self.t_m1

    x_p_t, p_p_t = self._predict_current_state(control_input, t)

    self.t_m1 = t
    self.x_t_m1 = x_p_t
    self.p_t_m1 = p_p_t

    return x_p_t


  def calc_previous_state_transform(self, u_t, dt, t):
    unimplemented()

  
  def calc_control_input_transform(self, u_t, dt, t):
    unimplemented()


  def calc_predicted_state_noise(self, u_t, dt, t):
    unimplemented()


  def calc_process_noise_covariance(self, u_t, dt, t):
    unimplemented()


  def calc_measurement_noise_covariance(self, y_t, dt, t):
    unimplemented()


  def calc_measurement_noise(self, y_t, dt, t):
    unimplemented()


  def _predict_current_state(self, u_t, t):
    dt = t - self.t_m1
    a = self.calc_previous_state_transform(u_t, dt, t)
    b = self.calc_control_input_transform(u_t, dt, t)
    w_t = self.calc_predicted_state_noise(u_t, dt, t)
    q_t = self.calc_process_noise_covariance(u_t, dt, t)

    x_p_t = a.dot(self.x_t_m1) + b.dot(u_t) + w_t
    p_p_t = a.dot(self.p_t_m1).dot(a.T) + q_t

    return x_p_t, p_p_t


  def _measure_current_state(self, measurement, t):
    dt = t - self.t_m1
    return self.c.dot(measurement) + self.calc_measurement_noise(measurement, dt, t)


  def _combine_prediction_and_measurement(self, prediction, measurement, t):
    dt = t - self.t_m1
    x_p_t, p_p_t = prediction
    r = self.calc_measurement_noise_covariance(measurement, dt, t)

    k = (p_p_t.dot(self.h)).dot(np.linalg.inv(self.h.dot(p_p_t).dot(self.h.T) + r))
    x_t = x_p_t + k.dot(measurement - self.h.dot(x_p_t))

    i = np.identity(self.p_t_m1.shape[0])
    p_t = (i - k.dot(self.h)).dot(p_p_t)

    return x_t, p_t



class SimpleFilter(object):

  def __init__(self, alpha, initial_state):
    self._x_t_m1 = initial_state
    self._t_m1 = time.time()
    self._alpha = alpha


  def calc_prediction(self, control_input, dt, t):
    unimplemented()

  
  def calc_measurement(self, measurement, dt, t):
    unimplemented()


  def predict(self, control_input, t):
    dt = t - self._t_m1
    x_t = self.calc_prediction(control_input, dt, t)
    
    self._x_t_m1 = x_t
    self._t_m1 = t

    return x_t


  def update(self, measurement, t):
    dt = t - self._t_m1
    x_t = self.calc_measurement(measurement, dt, t)
    x_t = self._x_t_m1 + (x_t - self._x_t_m1) * self._alpha

    self._x_t_m1 = x_t
    self._t_m1 = t


  @property
  def state(self):
    return self._x_t_m1
