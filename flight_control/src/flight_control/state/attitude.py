import time
from threading import Thread

import numpy as np
from wrapt import synchronized

from filters import KalmanFilter
from base import Sensor, TopicCombiner, DataAccumulator
from ..math.vector import vector_normalize
from ..math.quaternion import quaternion_to_dcm, quaternion_multiply, \
  quaternion_conjugate, quaternion_make, quaternion_rotate, quaternion_normalize
  

_PROCESS_ERROR_COVARIANCE_TRANSFORM = np.identity(4)
_MEASUREMENT_TRANSFORM = np.identity(4)

_INITIAL_STATE = np.array([1, 0, 0, 0])
_INITIAL_PROCESS_COVARIANCE = np.identity(4) * 10


class AttitudeSensor(Sensor):

  def __init__(self, meas_acc_gain, meas_mag_gain, std_gyro, std_acc, std_mag):
    super(AttitudeSensor, self).__init__()

    self._measurement_combiner = TopicCombiner(
      self._handle_measurement, ['accelerometer', 'compass'])

    self._measurement_accumulator = DataAccumulator()
    self._control_accumulator = DataAccumulator()

    self._filter = AttitudeKF(meas_acc_gain, meas_mag_gain, std_gyro, std_acc, std_mag)
    self._q = _INITIAL_STATE

  
  def post_accelerometer(self, measurement):
    self._measurement_combiner.post('accelerometer', measurement)


  def post_gyro(self, measurement):
    self._control_accumulator.post(measurement)


  def post_magnetometer(self, measurement):
    self._measurement_combiner.post('compass', measurement)


  def _handle_measurement(self, accelerometer, compass):
    self._measurement_accumulator.post(np.array([accelerometer, compass]).ravel())


  def _prediction_worker(self):
    control = self._control_accumulator.get()

    if control is None:
      return
    
    self._set_state(self._filter.predict(control, time.time()))


  def _measurement_worker(self):
    measurement = self._measurement_accumulator.get()

    if measurement is None:
      return
    
    self._set_state(self._filter.update(measurement, time.time()))


  def _worker_cycle(self):
    self._measurement_worker()

    for _ in range(10):
      self._prediction_worker()


  def _worker(self):
    while not self._thread is None:
      self._worker_cycle()


  def start(self):
    self._thread = Thread(target=self._worker)
    self._thread.start()


  def stop(self):
    self._thread = None


  @synchronized
  def get_state(self):
    return self._q


  @Sensor.mutator
  @synchronized
  def _set_state(self, state):
    self._q = state
  


class AttitudeKF(KalmanFilter):
  
  def __init__(self, meas_acc_gain, meas_mag_gain, std_gyro, std_acc, std_mag):
    super(AttitudeKF, self).__init__(
      _PROCESS_ERROR_COVARIANCE_TRANSFORM,
      _MEASUREMENT_TRANSFORM,
      _INITIAL_STATE, 
      _INITIAL_PROCESS_COVARIANCE
    )

    self._meas_acc_gain = meas_acc_gain
    self._meas_mag_gain = meas_mag_gain
    self._cov_gyro = np.diag(std_gyro**2)
    self._cov_meas = np.diag(np.array([std_acc, std_gyro]).ravel()**2)


  @classmethod
  def _rotate_q(cls, q, v_n, v_b, gain):
    v_n_est = vector_normalize(quaternion_rotate(q, v_b))
    v_n_est_error = np.arccos(v_n.T.dot(v_n_est))

    if v_n_est_error == 0:
      return q

    v_rot_axis = vector_normalize(np.cross(v_n_est, v_n))

    q_error = quaternion_make(
      np.cos(v_n_est_error * gain / 2),
      v_rot_axis * np.sin(v_n_est_error * gain / 2)
    )

    return quaternion_multiply(q_error, q)


  def update(self, measurement, t):
    ax, ay, az, mx, my, mz = measurement
    g_b = np.array([ax, ay, az]).T
    m_b = np.array([mx, my, mz]).T

    g_n = np.array([0, 0, -1]).T
    m_n = np.array([1, 0, 0]).T

    x_t_m1 = self.x_t_m1
    q_g = AttitudeKF._rotate_q(x_t_m1, g_n, g_b, self._meas_acc_gain)

    d_b = quaternion_rotate(quaternion_conjugate(q_g), g_n)
    e_b = vector_normalize(np.cross(d_b, m_b))
    n_b = np.cross(e_b, d_b)

    q_m = AttitudeKF._rotate_q(q_g, m_n, n_b, self._meas_mag_gain)

    return super(AttitudeKF, self).update(q_m, t)


  def predict(self, control_input, t):
    result = super(AttitudeKF, self).predict(control_input, t)
    self.x_t_m1 = quaternion_normalize(self.x_t_m1)
    return result


  def calc_previous_state_transform(self, u_t, dt, t):
    return np.identity(4)

  
  def calc_control_input_transform(self, u_t, dt, t):
    q0, q1, q2, q3 = self.x_t_m1
    return np.array([
      [-q1, -q2, -q3],
      [ q0, -q3,  q2],
      [ q3,  q0, -q1],
      [-q2,  q1,  q0]
    ]) * dt / 2


  def calc_predicted_state_noise(self, u_t, dt, t):
    # TODO bias
    return 0


  def calc_process_noise_covariance(self, u_t, dt, t):
    q0, q1, q2, q3 = self.x_t_m1
    g = np.array([
      [ q1,  q2,  q3],
      [-q0,  q3, -q2],
      [-q3, -q0,  q1],
      [ q2, -q1, -q0]
    ])

    return (dt**2 / 4) * g.dot(self._cov_gyro).dot(g.T)


  def calc_measurement_noise_covariance(self, y_t, dt, t):
    # TODO calculate correct value from accelerometer and gyro sensor std deviation
    return np.array([
      [0.01, 0.0, 0.0, 0.0],
      [0.0, 0.01, 0.0, 0.0],
      [0.0, 0.0, 0.01, 0.0],
      [0.0, 0.0, 0.0, 0.01]
    ])**2


  def calc_measurement_noise(self, y_t, dt, t):
    # TODO bias
    return 0
