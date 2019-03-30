import numpy as np

class SensorParams(object):

  def __init__(self):
    self.gyro_std = np.array([0.0016, 0.0016, 0.0016])
    self.gyro_bias = np.array([0, 0, 0])

    self.acc_std = np.array([0.042, 0.042, 0.042])
    self.acc_bias = np.array([0, 0, 0])

    self.compass_std = np.array([0.0055, 0.0055, 0.005])
    self.compass_bias = np.array([0, 0, 0])

    self.attitude_meas_acc_gain = 0.25
    self.attitude_meas_compass_gain = 0.5

    self.velocity_alpha = np.array([0.3, 0.3, 0.3, 0.5, 0.5, 0.5])