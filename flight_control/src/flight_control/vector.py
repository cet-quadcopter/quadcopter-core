import numpy as np


def vector_norm(v):
  v1, v2, v3 = v

  return (v1**2 + v2**2 + v3**2)**0.5


def vector_normalize(v):
  return v / vector_norm(v)