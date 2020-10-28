#!/usr/bin/env python
import sys
import getopt

import roslib
import rosbag
import rospy
from std_msgs.msg import String


def print_usage():
    print('Usage: restamp_bag.py <input_file>')
    print('Example: restamp_bag.py bursty_data.bag')
    sys.exit(2)


if __name__ == "__main__":
    if len(sys.argv) != 2:
	print_usage()
    in_path = sys.argv[1]
    out_path = in_path.replace(".bag", "-restamped.bag")

    print("Restamping bag [%s] to [%s]" % (in_path, out_path))
    print("This may take a while")

    in_bag = rosbag.Bag(in_path, 'r')
    out_bag = rosbag.Bag(out_path, 'w')
    msg_counter = 0
    for topic, msg, t in in_bag.read_messages():
        out_bag.write(topic, msg, msg.header.stamp)
        if (msg_counter % 100) == 0:
            sys.stdout.write('.')
            sys.stdout.flush()
        msg_counter = msg_counter + 1

    print("Done!")
    out_bag.close()
