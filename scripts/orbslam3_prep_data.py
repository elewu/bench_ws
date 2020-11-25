#!/usr/bin/env python
import sys

from numpy import genfromtxt

from tf import *


def print_usage():
    print("orbslam3_prep_data.py <est_file> <gnd_file> <save_file>")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print_usage()

    est_file = sys.argv[1]
    gnd_file = sys.argv[2]
    save_file = sys.argv[3]

    gnd_data = genfromtxt(gnd_file, delimiter=' ')
    ts = gnd_data[0][0] * 1e-9
    rx = gnd_data[0][1]
    ry = gnd_data[0][2]
    rz = gnd_data[0][3]
    qx = gnd_data[0][4]
    qy = gnd_data[0][5]
    qz = gnd_data[0][6]
    qw = gnd_data[0][7]

    r = np.array([rx, ry, rz])
    q = np.array([qw, qx, qy, qz])
    origin = tf(q, r)

    # Convert ORB SLAM3 output to rpg_evaluate_trajectory format
    save_file = open(save_file, "w")
    save_file.write("# timestamp tx ty tz qx qy qz qw\n")
    est_data = genfromtxt(est_file, delimiter=' ')
    for line in est_data:
        ts = line[0] * 1e-9
        rx = line[1]
        ry = line[2]
        rz = line[3]
        qx = line[4]
        qy = line[5]
        qz = line[6]
        qw = line[7]

        r = np.array([rx, ry, rz])
        q = np.array([qw, qx, qy, qz])
        pose = origin.dot(tf(q, r))

        pos = tf_trans(pose)
        quat = tf_quat(pose)

        save_file.write("%.18e " % ts)
        save_file.write("%.18e " % (pos[0]))
        save_file.write("%.18e " % (pos[1]))
        save_file.write("%.18e " % (pos[2]))
        save_file.write("%.18e " % (quat[1]))
        save_file.write("%.18e " % (quat[2]))
        save_file.write("%.18e " % (quat[3]))
        save_file.write("%.18e\n" % (quat[0]))
    save_file.close()
