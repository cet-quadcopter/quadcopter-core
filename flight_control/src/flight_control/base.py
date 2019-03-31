from wrapt import synchronized
import numpy as np



class PIDControl(object):

  def __init__(self, kp, ki, kd, num_integration_steps=10):
    assert kp.shape == ki.shape == kd.shape
    assert num_integration_steps >= 0

    self._kp = kp
    self._ki = ki
    self._kd = kd

    self._integrator = Integrator(num_integration_steps)
    self._differentiator = Differentiator()


  def update(self, error):
    assert error.shape == self._kp.shape
    
    e_i = self._integrator.add(error)
    e_d = self._differentiator.add(error)

    return self._kp * error + self._ki * e_i + self._kd * e_d
  


"""
Use only with fixed interval
"""
class Integrator(object):

  def __init__(self, limit):
    assert limit >= 2

    self._values = [0] * limit
    self._pointer = 0
    self._sum = 0


  @synchronized
  def add(self, val):
    replace = self._values[self._pointer]
    self._values[self._pointer] = val
    self._sum = val + self._sum - replace

    self._pointer = self._pointer + 1 if self._pointer < len(self._values) - 1 else 0

    return self._sum


"""
Use only with fixed interval
"""
class Differentiator(object):

  def __init__(self):
    self._pv = 0


  @synchronized
  def add(self, val):
    result = val - self._pv
    self._pv = val

    return result
