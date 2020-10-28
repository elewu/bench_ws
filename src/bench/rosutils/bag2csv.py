#!/usr/bin/env python
import sys
from math import atan2
from math import asin
from math import degrees

import rosbag
import rospy


def print_usage():
    print("Usage: bag2csv.py <ros bag> <ros topic> <output path>")
    print("Example: bag2csv.py record.bag /robot/pose robot_images.csv")


def quat2euler(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    qw2 = qw * qw
    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz

    x = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
    y = asin(2 * (qy * qw - qx * qz))
    z = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

    return [x, y, z]


class std_msgs:
    supported_msgs = [
        "std_msgs/Header"
        "std_msgs/String"
    ]

    @staticmethod
    def header_to_str(msg):
        header = "seq,frame_id,secs,nsecs"

        seq = msg.seq
        frame_id = msg.frame_id
        secs = msg.stamp.secs
        nsecs = msg.stamp.nsecs
        timestamp = rospy.Time(secs, nsecs)

        ts = str(timestamp.to_nsec())
        secs = str(ts[0:10])
        nsecs = str(ts[10:19])

        data = ",".join([str(seq), str(frame_id), secs, nsecs])
        return (header, data)

    @staticmethod
    def string_to_str(field_name, data):
        header = field_name
        return (header, data)

class geometry_msgs:
    supported_msgs = [
        "geometry_msgs/Point",
        "geometry_msgs/Vector3",
        "geometry_msgs/Quaternion",
        "geometry_msgs/Pose",
        "geometry_msgs/PoseStamped",
        "geometry_msgs/PoseWithCovarianceStamped"
        "geometry_msgs/Twist"
        "geometry_msgs/TwistStamped"
        "geometry_msgs/TwistWithCovarianceStamped"
    ]

    @staticmethod
    def point_to_str(msg, prefix=""):
        axis = ["x", "y", "z"]
        header = ",".join([prefix + ax for ax in axis])
        data = ",".join([str(msg.x), str(msg.y), str(msg.z)])
        return (header, data)

    @staticmethod
    def vector3_to_str(msg, prefix=""):
        axis = ["x", "y", "z"]
        header = ",".join([prefix + ax for ax in axis])
        data = ",".join([str(msg.x), str(msg.y), str(msg.z)])
        return (header, data)

    @staticmethod
    def quaternion_to_str(msg):
        header = "qw,qx,qy,qz,roll,pitch,yaw"
        rpy = quat2euler([msg.w, msg.x, msg.y, msg.z])
        data = ",".join([str(msg.w), str(msg.x), str(msg.y), str(msg.z),
                         str(degrees(rpy[0])), str(degrees(rpy[1])), str(degrees(rpy[2]))])
        return (header, data)

    @staticmethod
    def covariance_to_str(covar_data, prefix=""):
        header = ",".join([prefix + "covar_" + str(i) for i in range(36)])
        data = ",".join([str(x) for x in covar_data])
        return (header, data)

    @staticmethod
    def pose_to_str(msg):
        pos_header, pos_data = geometry_msgs.point_to_str(msg.position)
        rot_header, rot_data = geometry_msgs.quaternion_to_str(msg.orientation)

        header = pos_header + "," + rot_header
        data = pos_data + "," + rot_data
        return (header, data)

    @staticmethod
    def pose_with_covariance_to_str(msg):
        pose_header, pose_data = geometry_msgs.pose_to_str(msg.pose)
        covar_header, covar_data = geometry_msgs.covariance_to_str(msg.covariance, "pose_")

        header = pose_header + "," + covar_header
        data = pose_data + "," + covar_data
        return (header, data)

    @staticmethod
    def pose_stamped_to_str(msg):
        msg_header, header_data = std_msgs.header_to_str(msg.header)
        pose_header, pose_data = geometry_msgs.pose_to_str(msg.pose)

        header = msg_header + "," + pose_header
        data = header_data + "," + pose_data
        return (header, data)

    @staticmethod
    def pose_with_covariance_stamped_to_str(msg):
        msg_header, header_data = std_msgs.header_to_str(msg.header)
        pose_header, pose_data = geometry_msgs.pose_with_covariance_to_str(msg.pose, "pose_")

        header = msg_header + "," + pose_header
        data = header_data + "," + pose_data
        return (header, data)

    @staticmethod
    def twist_to_str(msg):
        linear_header, linear_data = geometry_msgs.vector3_to_str(msg.linear, "a")
        angular_header, angular_data = geometry_msgs.vector3_to_str(msg.angular, "w")

        header = linear_header + "," + angular_header
        data = linear_data + "," + angular_data
        return (header, data)

    @staticmethod
    def twist_with_covariance_to_str(msg):
        twist_header, twist_data = geometry_msgs.twist_to_str(msg.twist)
        covar_header, covar_data = geometry_msgs.covariance_to_str(msg.covariance, "twist_")

        header = twist_header + "," + covar_header
        data = twist_data + "," + covar_data
        return (header, data)

    @staticmethod
    def twist_stamped_to_str(msg):
        msg_header, header_data = std_msgs.header_to_str(msg.header)
        twist_header, twist_data = geometry_msgs.twist_to_str(msg.twist)

        header = msg_header + "," + twist_header
        data = msg_data + "," + twist_data
        return (header, data)

    @staticmethod
    def twist_with_covariance_stamped_to_str(msg):
        msg_header, header_data = std_msgs.header_to_str(msg.header)
        twist_header, twist_data = geometry_msgs.twist_to_str(msg.twist)
        covar_header, covar_data = geometry_msgs.covariance_to_str(msg.covariance, "twist_")

        header = msg_header + "," + twist_header + "," + covar_header
        data = msg_data + "," + twist_data + "," + covar_data
        return (header, data)

class nav_msgs:
    supported_msgs = [
        "nav_msgs/Odometry",
    ]

    @staticmethod
    def odometry_to_str(msg):
        msg_header, msg_data = std_msgs.header_to_str(msg.header)
        _, str_data = std_msgs.string_to_str("child_frame_id", msg.child_frame_id)
        pose_header, pose_data = geometry_msgs.pose_with_covariance_to_str(msg.pose)
        twist_header, twist_data = geometry_msgs.twist_with_covariance_to_str(msg.twist)

        header = msg_header + "," + "child_frame_id" + "," + pose_header + "," + twist_header
        data = msg_data + "," + str_data + "," + pose_data + "," + twist_data
        return (header, data)

class sensor_msgs:
    supported_msgs = [
        "sensor_msgs/Imu"
    ]

    @staticmethod
    def imu_to_str(msg):
        msg_header, msg_data = std_msgs.header_to_str(msg.header)
        _, gyr = geometry_msgs.vector3_to_str(msg.angular_velocity)
        _, acc = geometry_msgs.vector3_to_str(msg.linear_acceleration)
        header = msg_header + ",wx,wy,wz,ax,ay,az"
        data = msg_data + "," + gyr + "," + acc
        return (header, data)


def check_topic_exists(bag, topic):
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")


def check_topic_type(bag, topic):
    info = bag.get_type_and_topic_info()
    msg_type = info.topics[topic].msg_type
    supported_msgs = std_msgs.supported_msgs
    supported_msgs += geometry_msgs.supported_msgs
    supported_msgs += nav_msgs.supported_msgs
    supported_msgs += sensor_msgs.supported_msgs

    if msg_type not in supported_msgs:
        supported_list = ""
        for x in supported_msgs:
            supported_list += "  - " + str(x) + "\n"

        err_msg = "bag2csv does not support msg type: [%s]\n" % msg_type
        err_msg += "bag2csv currently only supports:\n%s" % supported_list
        raise RuntimeError(err_msg)


def get_msg_converter(bag, topic):
    info = bag.get_type_and_topic_info()
    msg_type = info.topics[topic].msg_type

    # STD MSGS
    if msg_type == "std_msgs/Header":
        return std_msgs.header_to_str

    # GEOMETRY MSGS
    if msg_type == "geometry_msgs/Point":
        return geometry_msgs.point_to_str
    if msg_type == "geometry_msgs/Vector3":
        return geometry_msgs.vector3_to_str
    if msg_type == "geometry_msgs/Quaternion":
        return geometry_msgs.quaternion_to_str
    if msg_type == "geometry_msgs/Pose":
        return geometry_msgs.pose_to_str
    if msg_type == "geometry_msgs/PoseStamped":
        return geometry_msgs.pose_stamped_to_str
    if msg_type == "geometry_msgs/PoseWithCovarianceStamped":
        return geometry_msgs.pose_with_covariance_stamped_to_str

    # ODOMETRY MSGS
    if msg_type == "nav_msgs/Odometry":
        return nav_msgs.odometry_to_str

    # SENSOR MSGS
    if msg_type == "sensor_msgs/Imu":
        return sensor_msgs.imu_to_str


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
