#!/usr/bin/env python
import sys

import numpy as np
import cv2
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# DEPTH_TOPIC = "/camera/depth/image_rect_raw"
# DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
# RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/rs/depth0/image"
RGB_TOPIC = "/rs/rgb0/image"

depth_image = None
rgb_image = None

def cvt8bit(image):
    max_m = 5000  # 5m. Double check your sensor specs
    cv_image_tmp = np.clip(image, 0, max_m)  # Filter all values > max_m
    scale_factor = (255.0) / max_m
    image = image * scale_factor
    return np.uint8(image)


def depth_callback(data):
    cv_bridge = CvBridge()
    image = cv_bridge.imgmsg_to_cv2(data)
    image = cvt8bit(image)
    global depth_image
    depth_image = cv2.applyColorMap(image, cv2.COLORMAP_JET)


def rgb_callback(data):
    cv_bridge = CvBridge()
    global rgb_image
    rgb_image = cv_bridge.imgmsg_to_cv2(data, "bgr8")

    if depth_image is not None:
        image = np.hstack((depth_image, rgb_image))
        cv2.imshow("Image", image)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback)
    rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, rgb_callback)
    rospy.spin()
