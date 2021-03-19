#!/usr/bin/env python

import rospy
import cv2
import numpy as np
# from std_msgs.msg import String
# from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global out, c, frame_num, fourcc, img
img = [] 
frame_num = 5000
c = 0

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'DIVX')

def out_fun(shape):
    global out, fourcc
    is_color = False
    out = cv2.VideoWriter('video.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), is_color)

def _shutdown():
    global out, c, img
    out.release()
    print c, " frames has been captured"
    rospy.loginfo("total time: " + str(time.now().secs) + " [sec]")
    print "shutdown"


def _Img_callback(ros_data, bridge):
    global out, c, start, time

    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    try:
        # encoding of simulated depth image: 32FC1, encoding of real depth image: 16UC1
        cv_image = bridge.imgmsg_to_cv2(ros_data, desired_encoding="32FC1")
    except CvBridgeError as e:
      print(e)
    cv_image = np.array(cv_image, dtype=np.float)
    cv_image = cv2.normalize(cv_image, cv_image, 0, 255, cv2.NORM_MINMAX)
    cv_image = np.round(cv_image).astype(np.uint8)

    if c == 0:
        time = rospy.Time(0)
        start = rospy.Time.now().nsecs
        frame_shape = np.shape(cv_image)
        out_fun(frame_shape)

    #frame = cv2.flip(frame,0)
    out.write(cv_image)

    # img.append(cv_image)
    # rospy.sleep(0.1)
    current = rospy.Time.now().nsecs
    dt = (current - start) * 1e-9
    start = current
    rospy.loginfo("dt " + str(dt) + " [sec]")
    rospy.loginfo("frame rate " + str(1/dt) + " [Hz]")
    c = 1+c

    print "The number amount of captured frames: ", c
    if (c == frame_num):
        print "Video end "
        rospy.signal_shutdown("Video end")


rospy.init_node('video_handler', anonymous=True)
bridge = CvBridge()
rospy.on_shutdown(_shutdown)

rospy.Subscriber("/kinect/depth/image_raw",
                         Image, lambda img: _Img_callback(img, bridge), queue_size=2)
rospy.wait_for_message("/kinect/depth/image_raw", Image)

rospy.spin()
