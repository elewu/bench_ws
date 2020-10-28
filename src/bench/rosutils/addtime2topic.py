#!/usr/bin/env python
import os
import sys

import rosbag


def print_usage():
    print("Usage: addtime2topic.py <ros bag> <topic>")
    print("Usage: addtime2topic.py recorded.bag /robot/imu")


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 3:
        print_usage()
        exit(-1)

    bag_path = sys.argv[1]
    target_topic = sys.argv[2]
    repair_path = bag_path.replace(".bag", "-repaired.bag")
    print("Output saved to [%s]" % repair_path)

    # Open bag
    bag = rosbag.Bag(bag_path, 'r')
    fix_bag = rosbag.Bag(repair_path, "w")

    # Check if topic is in bag
    info = bag.get_type_and_topic_info()
    if target_topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")

    # Iterate through bag
    for topic, msg, t in bag.read_messages():
        # Add time back to message header
        if topic == target_topic:
            msg.header.stamp.secs = t.secs
            msg.header.stamp.nsecs = t.nsecs
        fix_bag.write(topic, msg, t)

    # Close bag - very important else bag is invalid
    fix_bag.close()
