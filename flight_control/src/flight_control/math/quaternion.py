import numpy as np


QUATERNION_IDENTITY = np.array([1, 0, 0, 0])


def quaternion_multiply(p, q):
  p0, p1, p2, p3 = p
  q0, q1, q2, q3 = q

  return np.array([
    p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
    p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
    p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
    p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0
  ])


def quaternion_conjugate(q):
  q0, q1, q2, q3 = q

  return np.array([q0, -q1, -q2, -q3])


def quaternion_norm(q):
  q0, q1, q2, q3 = q

  return np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)


def quaternion_inverse(q):
  return quaternion_conjugate(q) / quaternion_norm(q)


def quaternion_normalize(q):
  return q / quaternion_norm(q)


def quaternion_to_dcm(q):
  q0, q1, q2, q3 = q
  return np.array([
    [q0**2 + q1**2 - q2**2 - q3**2, 2 * q0 * q3 + 2 * q1 * q2    , -2 * q0 * q2 + 2 * q1 * q3   ],
    [-2 * q0 * q3 + 2 * q1 * q2   , q0**2 - q1**2 + q2**2 - q3**2, 2 * q0 * q1 + 2 * q2 * q3    ],
    [2 * q0 * q2 + 2 * q1 * q3    , -2 * q0 * q1 + 2 * q2 * q3   , q0**2 - q1**2 - q2**2 + q3**2]
  ])


def quaternion_to_euler_123(q):
  q0, q1, q2, q3 = q

  psi = np.arctan2(2 * q2 * q3 + 2 * q0 * q1, q3**2 - q2**2 - q1**2 + q0**2)
  theta = -np.arcsin(2 * q1 * q3 - 2 * q0 * q2)
  gamma = np.arctan2(2 * q1 * q2 + 2 * q0 * q3, q1**2 + q0**2 - q3**2 - q2**2)

  return np.array([psi, theta, gamma]) * 180 / np.pi


def quaternion_make(real, imaginary):
  im1, im2, im3 = imaginary
  return np.array([real, im1, im2, im3])


def quaternion_rotate(q, v):
  v = quaternion_make(0, v)
  return quaternion_multiply(quaternion_multiply(q, v), quaternion_conjugate(q))[1:]
