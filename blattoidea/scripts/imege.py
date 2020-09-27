#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

global frame

def shutdown_callback():
    cv2.destroyAllWindows()

def img_process():
    global frame
    light_orange = (0, 10, 0)
    dark_orange = (50, 255, 255)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, light_orange, dark_orange)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("mask",mask)
    cv2.imshow("hsv", hsv)
    cv2.waitKey(1) 

def img_callback(ros_img):
    global frame
    np_arr = np.fromstring(ros_img.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def main():
    rospy.init_node('img_processing', anonymous=True)
    rospy.Subscriber('/kinect/color/image_raw/compressed', CompressedImage, img_callback, queue_size=2)
    rospy.wait_for_message('/kinect/color/image_raw/compressed', CompressedImage)
    rospy.on_shutdown(shutdown_callback)

if __name__ == "__main__":
    main()
    while not rospy.is_shutdown():
        img_process()
    