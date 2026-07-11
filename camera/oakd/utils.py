import numpy as np

def quat_rotate(qx, qy, qz, qw, v):
    q = np.array([qx, qy, qz])
    t = 2.0 * np.cross(q, v)
    return v + qw * t + np.cross(q, t)
