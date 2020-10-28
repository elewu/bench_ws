#!/usr/bin/env python
import sys

import cv2
import rosbag
from cv_bridge import CvBridge


def print_usage():
    print("Usage: bag2vid.py <ros bag> <image topic> <video output path>")
    print("Example: bag2vid.py test.bag /camera/image video.avi")


def bag2vid(bag_path, topic, output_path, preview=False):
    bag = rosbag.Bag(bag_path, 'r')

    # Check if topic is in bag
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")

    # Check image message type
    msg_type = info.topics[topic].msg_type
    supported_msgs = ["sensor_msgs/CompressedImage", "sensor_msgs/Image"]
    if msg_type not in supported_msgs:
        err_msg = "Script only supports %s!" % " or ".join(supported_msgs)
        raise RuntimeError(err_msg)

    # Get image shape
    image_shape = None
    br = CvBridge()
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if msg_type == "sensor_msgs/CompressedImage":
            image = br.compressed_imgmsg_to_cv2(msg)
            image_shape = image.shape
        else:
            image = br.imgmsg_to_cv2(msg)
            image_shape = image.shape
        break

    # Create video writer
    height = image_shape[0]
    width = image_shape[1]
    shape = (width, height)
    encoding = cv2.VideoWriter_fourcc(*"MJPG")
    fps = info.topics[topic].frequency
    video = cv2.VideoWriter(output_path, encoding, fps, shape)

    # Write out the video
    index = 0.0
    index_end = float(info.topics[topic].message_count)

    for topic, msg, t in bag.read_messages(topics=[topic]):
        if msg_type == "sensor_msgs/CompressedImage":
            image = br.compressed_imgmsg_to_cv2(msg)
            video.write(image)
        else:
            image = br.imgmsg_to_cv2(msg, "bgr8")
            video.write(image)

        # Preview image
        if preview:
            cv2.imshow("Video: " + topic, image)
            cv2.waitKey(1)

        # Print progress
        p = int((index / index_end) * 100.0)
        print("Converting topic [%s] in bag [%s] to video - progress: %.0f%%"
              % (topic, bag_path, p))
        sys.stdout.write("\033[F")  # Move cursor up one line
        sys.stdout.flush()
        index += 1.0

    #  Print final progress
    print("Converting topic [%s] in bag [%s] to video - progress: %.0f%%"
          % (topic, bag_path, p))
    video.release()


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print_usage()
        exit(-1)

    bag_path = sys.argv[1]
    topic = sys.argv[2]
    output_path = sys.argv[3]
    bag2vid(bag_path, topic, output_path)
