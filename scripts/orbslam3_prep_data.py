#!/usr/bin/env python
import sys

from numpy import genfromtxt


def print_usage():
    print("orbslam3_prep_data.py <infile> <outfile>")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print_usage()

    infile = sys.argv[1]
    outfile = sys.argv[2]

    data = genfromtxt(infile, delimiter=' ')
    outfile = open(outfile, "w")
    outfile.write("# timestamp tx ty tz qx qy qz qw\n")
    for line in data:
        outfile.write("%.18e " % (line[0] * 1e-9))
        outfile.write("%.18e " % (line[1]))
        outfile.write("%.18e " % (line[2]))
        outfile.write("%.18e " % (line[3]))
        outfile.write("%.18e " % (line[4]))
        outfile.write("%.18e " % (line[5]))
        outfile.write("%.18e " % (line[6]))
        outfile.write("%.18e\n" % (line[7]))
    outfile.close()
