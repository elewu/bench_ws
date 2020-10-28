#!/usr/bin/env python
import sys

import numpy as np
import cv2
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEPTH_TOPIC = "/my_depth_camera/depth"


def cvt8bit(image):
    max_m = 5000  # 5m. Double check your sensor specs
    cv_image_tmp = np.clip(image, 0, max_m)  # Filter all values > max_m
    scale_factor = (255.0) / max_m
    image = image * scale_factor
    return np.uint8(image)


def depth_callback(data):
    cv_bridge = CvBridge()
    image = cv_bridge.imgmsg_to_cv2(data, "32FC1")
    print("shape", image.shape)
    print("min", np.min(image))
    print("max", np.max(image))

    # image = cvt8bit(image)
    # image = cv2.applyColorMap(image, cv2.COLORMAP_JET)

    if image is not None:
        cv2.imshow("Image", image / np.linalg.norm(image))
        cv2.waitKey(1)
    # image = cvt8bit(image)
    # global depth_image
    # depth_image = cv2.applyColorMap(image, cv2.COLORMAP_JET)

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback)
    # rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, rgb_callback)
    rospy.spin()
