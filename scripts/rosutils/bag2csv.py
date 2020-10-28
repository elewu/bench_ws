#!/usr/bin/env python
import sys
from math import atan2
from math import asin
from math import degrees

import rosbag
import rospy
from rosutils import *


def print_usage():
    print("Usage: bag2csv.py <ros bag> <ros topic> <output path>")
    print("Example: bag2csv.py record.bag /robot/pose robot_images.csv")


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 4:
        print_usage()
        exit(-1)

    # Parse CLI args
    bag_path = sys.argv[1]
    topic = sys.argv[2]
    output_path = sys.argv[3]
    bag = rosbag.Bag(bag_path, 'r')

    # Checks
    check_topic_exists(bag, topic)
    check_topic_type(bag, topic)
    msg_converter = get_msg_converter(bag, topic)

    # Output csv file
    print("Processing rosbag: [%s]" % (bag_path))
    print("Extracting rostopic: [%s]" % (topic))
    print("Saving to: [%s]" % (output_path))
    # -- Output header
    csv_file = open(output_path, "w")
    topic, msg, t = next(bag.read_messages(topics=[topic]))
    header, data = msg_converter(msg)
    csv_file.write("#" + header + "\n")
    csv_file.write(data + "\n")
    # -- Output data
    for topic, msg, t in bag.read_messages(topics=[topic]):
        _, data = msg_converter(msg)
        csv_file.write(data + "\n")
        csv_file.flush()
    csv_file.close()
    print("Done!")
