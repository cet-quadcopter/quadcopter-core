from attitude import AttitudeSensor
from params import SensorParams


class StateManager(object):

  def __init__(self, params=SensorParams()):
    self._attitude_sensor = AttitudeSensor(
      params.attitude_meas_acc_gain,
      params.attitude_meas_compass_gain,
      params.gyro_std,
      params.acc_std,
      params.compass_std
    )


  def post_gyro(self, measurement):
    self._attitude_sensor.post_gyro(measurement)


  def post_accelerometer(self, measurement):
    self._attitude_sensor.post_accelerometer(measurement)


  def post_magnetometer(self, measurement):
    self._attitude_sensor.post_magnetometer(measurement)


  def post_gps(self, measurement):
    pass


  def start(self):
    self._attitude_sensor.start()


  def stop(self):
    self._attitude_sensor.stop()

  
  @property
  def attitude(self):
    return self._attitude_sensor.get_state()
