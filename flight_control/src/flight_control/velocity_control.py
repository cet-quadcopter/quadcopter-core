import numpy as np

from base import PIDControl
from math.vector import vector_norm, vector_normalize
from math.quaternion import quaternion_rotate, quaternion_make, quaternion_multiply

class VelocityControlParams(object):

  def __init__(self):
    self.force_kp = np.array([1, 1, 1]) * 10
    self.force_kd = np.array([0.5, 0.5, 0.5]) * -1
    self.force_ki = np.array([0.5, 0.5, 0.5]) * 0

    self.torque_kp = np.array([0.5, 0.5, 0.5]) * 0.0
    self.torque_kd = np.array([0.5, 0.5, 0.5]) * 0.0
    self.torque_ki = np.array([0.5, 0.5, 0.5]) * 0.0


  
class VelocityControl(object):

  def __init__(self, state_manager, params=VelocityControlParams()):
    self._state_manager = state_manager

    self._force_control = PIDControl(
      params.force_kp, params.force_ki, params.force_kd
    )

    self._torque_control = PIDControl(
      params.torque_kp, params.torque_ki, params.torque_kd
    )

    self._m = 2.5
    b = 30
    k = 2
    d = 0.15  

    self._a_inv = np.linalg.inv(np.array([
      [-b  , -b  , -b  , -b  ],
      [ 0  , -d*b,  0  ,  d*b],
      [ d*b,  0  , -d*b,  0  ],
      [ k  , -k  ,  k  , -k  ]
    ]))


  def calc_signal(self, vn, ve, vd, omegad):
    g_n = np.array([0, 0, -9.8]) # TODO dynamic gravity
    f_ext = np.array([0, 0, 0])

    v_c = self._state_manager.velocity[0:4]
    v_i = np.array([vn, ve, vd, omegad])
    v_e = v_i - v_c

    a_n = self._force_control.update(v_e[0:3])
    f_n = self._m * a_n + self._m * g_n + f_ext # TODO drag
    f_b = np.array([0, 0, f_n[2]])

    q_c = self._state_manager.attitude
    f_n_est = quaternion_rotate(q_c, f_b)
    f_n_err = np.arccos(f_n_est.T.dot(f_n) / (vector_norm(f_n) * vector_norm(f_n_est)))

    if f_n_err == 0:
      q_e_force = np.array([1, 0, 0, 0])
    else:
      f_n_rot_axis = vector_normalize(np.cross(f_n_est, f_n))
      q_e_force = quaternion_make(
        np.cos(f_n_err / 2),
        np.sin(f_n_err / 2) * f_n_rot_axis
      )

    q_e_omegad = np.array([1, 0, 0, 0]) # TODO implement

    q_e = quaternion_multiply(q_e_force, q_e_omegad)

    tau_x, tau_y, tau_z = self._torque_control.update(q_e[1:]) # TODO correct rotation
    _, _, thrust = f_b
    
    omega = np.sqrt(np.maximum(self._a_inv.dot(np.array([thrust, tau_x, tau_y, tau_z])), 0))

    return omega
