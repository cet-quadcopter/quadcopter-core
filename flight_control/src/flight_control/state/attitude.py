import numpy as np

from filters import KalmanFilter
from ..vector import vector_normalize
from ..quaternion import quaternion_to_dcm, quaternion_multiply, \
  quaternion_conjugate, quaternion_make, quaternion_rotate, quaternion_normalize

_MEASUREMENT_ACCELEROMETER_GAIN = 1
_MEASUREMENT_MAGNETOMETER_GAIN = 1

_PROCESS_ERROR_COVARIANCE_TRANSFORM = np.identity(4)
_MEASUREMENT_TRANSFORM = np.identity(4)

_GYRO_COVARIANCE = np.diag([0.0016, 0.0016, 0.0016])
_ACCELEROMETER_COVARIANCE = np.diag([0.042**2, 0.042**2, 0.042**2])
_MAGNETOMETER_COVARIANCE = np.diag([0.0055**2, 0.0055**2, 0.005**2])

_INITIAL_STATE = np.array([1, 0, 0, 0])
_INITIAL_PROCESS_COVARIANCE = np.identity(4) * 10


class AttitudeKF(KalmanFilter):
  
  def __init__(self):
    super(AttitudeKF, self).__init__(
      _PROCESS_ERROR_COVARIANCE_TRANSFORM,
      _MEASUREMENT_TRANSFORM,
      _INITIAL_STATE, 
      _INITIAL_PROCESS_COVARIANCE
    )


  @classmethod
  def _rotate_q(cls, q, v_n, v_b, gain):
    v_n_est = quaternion_rotate(q, vector_normalize(v_b))
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
    q_g = AttitudeKF._rotate_q(x_t_m1, g_n, g_b, _MEASUREMENT_ACCELEROMETER_GAIN)

    d_b = quaternion_rotate(quaternion_conjugate(q_g), g_n)
    e_b = vector_normalize(np.cross(d_b, m_b))
    n_b = np.cross(e_b, d_b)

    q_m = AttitudeKF._rotate_q(q_g, m_n, n_b, _MEASUREMENT_MAGNETOMETER_GAIN)

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

    return (dt**2 / 4) * g.dot(_GYRO_COVARIANCE).dot(g.T)


  def calc_measurement_noise_covariance(self, y_t, dt, t):
    # TODO calculate correct value from accelerometer and gyro sensor std deviation
    return np.array([
      [0.001, 0.0, 0.0, 0.0],
      [0.0, 0.001, 0.0, 0.0],
      [0.0, 0.0, 0.001, 0.0],
      [0.0, 0.0, 0.0, 0.001]
    ])


  def calc_measurement_noise(self, y_t, dt, t):
    # TODO bias
    return 0
