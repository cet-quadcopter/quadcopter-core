from attitude import AttitudeSensor
from velocity import VelocitySensor
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
    self._attitude_sensor.subscribe(self.post_attitude)

    self._velocity_sensor = VelocitySensor(
      params.velocity_alpha
    )


  def post_gyro(self, measurement):
    self._attitude_sensor.post_gyro(measurement)
    self._velocity_sensor.post_gyro(measurement)


  def post_accelerometer(self, measurement):
    self._attitude_sensor.post_accelerometer(measurement)
    self._velocity_sensor.post_accelerometer(measurement)


  def post_magnetometer(self, measurement):
    self._attitude_sensor.post_magnetometer(measurement)


  def post_gps(self, measurement):
    self._velocity_sensor.post_gps(measurement)


  def post_attitude(self, sensor):
    q = sensor.get_state()
    self._velocity_sensor.post_attitude(q)


  def start(self):
    self._attitude_sensor.start()
    self._velocity_sensor.start()


  def stop(self):
    self._attitude_sensor.stop()
    self._velocity_sensor.stop()

  
  @property
  def attitude(self):
    return self._attitude_sensor.get_state()


  @property
  def velocity(self):
    return self._velocity_sensor.get_state()
