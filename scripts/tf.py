from math import sqrt

import numpy as np


def quat2rot(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz
    qw2 = qw * qw

    # Homogeneous form
    R11 = qw2 + qx2 - qy2 - qz2
    R12 = 2 * (qx * qy - qw * qz)
    R13 = 2 * (qx * qz + qw * qy)

    R21 = 2 * (qx * qy + qw * qz)
    R22 = qw2 - qx2 + qy2 - qz2
    R23 = 2 * (qy * qz - qw * qx)

    R31 = 2 * (qx * qz - qw * qy)
    R32 = 2 * (qy * qz + qw * qx)
    R33 = qw2 - qx2 - qy2 + qz2

    return np.array([[R11, R12, R13],
		     [R21, R22, R23],
                     [R31, R32, R33]])


def rot2quat(R):
    m00 = R[0, 0]
    m01 = R[0, 1]
    m02 = R[0, 2]

    m10 = R[1, 0]
    m11 = R[1, 1]
    m12 = R[1, 2]

    m20 = R[2, 0]
    m21 = R[2, 1]
    m22 = R[2, 2]

    tr = m00 + m11 + m22

    if (tr > 0):
	S = sqrt(tr+1.0) * 2
	qw = 0.25 * S
	qx = (m21 - m12) / S
	qy = (m02 - m20) / S
	qz = (m10 - m01) / S
    elif ((m00 > m11) and (m00 > m22)):
	S = sqrt(1.0 + m00 - m11 - m22) * 2
	qw = (m21 - m12) / S
	qx = 0.25 * S
	qy = (m01 + m10) / S
	qz = (m02 + m20) / S
    elif (m11 > m22):
	S = sqrt(1.0 + m11 - m00 - m22) * 2
	qw = (m02 - m20) / S
	qx = (m01 + m10) / S
	qy = 0.25 * S
	qz = (m12 + m21) / S
    else:
	S = sqrt(1.0 + m22 - m00 - m11) * 2
	qw = (m10 - m01) / S
	qx = (m02 + m20) / S
	qy = (m12 + m21) / S
	qz = 0.25 * S

    return np.array([qw, qx, qy, qz])


def tf(rot, r):
    C = None
    if rot.shape == (3, 3):
        C = rot
    elif rot.shape == (4, 1) or rot.shape == (4,):
        C = quat2rot(rot)
    else:
        raise RuntimeError("Invalid rot shape {0}".format(rot.shape))

    T = np.eye(4, 4)
    T[0:3, 0:3] = C
    T[0:3, 3] = r
    return T


def tf_trans(T):
    return T[0:3, 3]


def tf_rot(T):
    return T[0:3, 0:3]


def tf_quat(T):
    return rot2quat(tf_rot(T))

def tf_vec(T):
    r = tf_trans(T)
    q = tf_quat(T)
    return np.array([r[0], r[1], r[2], q[0], q[1], q[2], q[3]])
