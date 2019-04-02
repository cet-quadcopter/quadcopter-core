import time
from threading import Thread

import numpy as np
from wrapt import synchronized

from base import Sensor, DataAccumulator
from filters import ComplementaryFilter
from ..math.quaternion import quaternion_rotate


_INITIAL_STATE = np.array([0, 0, 0, 0, 0, 0])


class VelocitySensor(Sensor):

  def __init__(self, alpha):
    super(VelocitySensor, self).__init__()

    self._filter = VelocityFilter(alpha)

    self._gyro_acc = DataAccumulator()
    self._accelerometer_acc = DataAccumulator()
    self._velocity_acc = DataAccumulator()

    self._q = np.array([1, 0, 0, 0])
    self._gyro = np.array([0, 0, 0])
    self._accelerometer = np.array([0, 0, -9.8])
    self._velocity = np.array([0, 0, 0])
    self._state = np.array([0, 0, 0, 0, 0, 0])

    self._thread = None
  
  
  def post_gyro(self, measurement):
    self._gyro_acc.post(measurement)


  def post_accelerometer(self, measurement):
    self._accelerometer_acc.post(measurement)


  def post_gps(self, measurement):
    vel = measurement[7:10] / 100
    self._velocity_acc.post(vel)


  def post_attitude(self, measurement):
    self._q = measurement


  def _get_gyro(self):
    gyro = self._gyro_acc.get()
    if gyro is not None:
      self._gyro = gyro

    return self._gyro


  def _get_accelerometer(self):
    acc = self._accelerometer_acc.get()
    if acc is not None:
      self._accelerometer = acc

    return self._accelerometer


  def _get_velocity(self):
    vel = self._velocity_acc.get()
    if vel is not None:
      self._velocity = vel

    return self._velocity

  
  def _get_attitude(self):
    return self._q


  def _worker_cycle(self):
    gyro = self._get_gyro()  
    accelerometer = self._get_accelerometer()
    velocity = self._get_velocity()
    attitude = self._get_attitude()

    omega_n = quaternion_rotate(attitude, gyro)
    acc_n = quaternion_rotate(attitude, accelerometer) - [0, 0, -9.8]  # FIXME dynamic gravity
    
    control_input = acc_n
    measurement = np.array([velocity, omega_n]).ravel()

    self._set_state(self._filter.update(control_input, measurement, time.time()))


  def _worker(self):
    while self._thread is not None:
      self._worker_cycle()
      time.sleep(0.01)
  
  
  def start(self):
    self._thread = Thread(target=self._worker)
    self._thread.start()

  
  def stop(self):
    self._thread = None

  
  @synchronized
  def _set_state(self, state):
    self._state = state


  @Sensor.mutator
  @synchronized
  def get_state(self):
    return self._state



class VelocityFilter(ComplementaryFilter):

  def __init__(self, alpha):
    super(VelocityFilter, self).__init__(alpha, _INITIAL_STATE)

    self._alpha_t_m1 = np.array([0, 0, 0])


  def calc_prediction(self, control_input, dt, t):
    acc_t = control_input

    v_t_m1 = self._x_t_m1[0:3]
    omega_t_m1 = self._x_t_m1[3:6]

    omega_pred = omega_t_m1 + self._alpha_t_m1 * dt
    v_pred = v_t_m1 + acc_t * dt

    return np.array([v_pred, omega_pred]).ravel()


  def update(self, control_input, measurement, t):
    dt = t - self._t_m1

    omega_t_m1 = self._x_t_m1[3:6]
    result = super(VelocityFilter, self).update(control_input, measurement, t)
    omega_t = result[3:6]

    self._alpha_t_m1 = (omega_t - omega_t_m1) / dt

    return result

