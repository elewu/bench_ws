#!/usr/bin/env python
import sys

import rosbag
import rospy

from nav_msgs.msg import Odometry

def print_usage():
    print("Usage: prep_data.py <ros bag> <topic> <output_path>")


def check_topic_exists(bag, topic):
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic [%s] not in bag!" % topic)


def parse_msg_type(msg):
    return str(msg._type)


def nav_msgs_odom_to_csv(msg, csv):
    ts = msg.header.stamp.to_sec()
    r = msg.pose.pose.position
    q = msg.pose.pose.orientation
    line = [ts, r.x, r.y, r.z, q.x, q.y, q.z, q.w]
    line = " ".join([format(x, '.18e') for x in line])
    csv.write(line + "\n")


def msg2csv(msg, csv):
    msg_type = parse_msg_type(msg)
    if msg_type == "nav_msgs/Odometry":
        nav_msgs_odom_to_csv(msg, csv)
    # else:
    #     raise RuntimeError("Unsupported msg type [%s]" % msg_type)


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
    check_topic_exists(bag, topic)

    # Convert messages to stamped data
    csv = open(output_path, "w")
    csv.write("# timestamp tx ty tz qx qy qz qw\n")
    for topic, msg, _ in bag.read_messages(topics=[topic]):
        msg2csv(msg, csv)
    csv.close()
