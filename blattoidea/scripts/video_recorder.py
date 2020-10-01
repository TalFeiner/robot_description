#!/usr/bin/env python

import rospy
import cv2
import numpy as np
# from std_msgs.msg import String
# from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import skvideo.io

global out, c, frame_num, fourcc, img
img = [] 
frame_num = 1000
c = 0

# Define the codec and create VideoWriter object
# fourcc = cv2.cv2.CV_8UC1

# def out_fun(shape):
#     global out, fourcc
#     out = cv2.VideoWriter('video.avi',fourcc, 20.0, (int(shape[1]), int(shape[0])))

def _shutdown():
    global out, c, img
    # out.release()
    # img = np.array(img).astype(np.uint8)
    # skvideo.io.vwrite("outputvideo.mp4", img)

    print c, " frames has been captured"
    print "shutdown"


def _Img_callback(ros_data, bridge, writer):
    global out, c

    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    try:
      cv_image = bridge.imgmsg_to_cv2(ros_data, "8UC1")
    except CvBridgeError as e:
      print(e)
    #     out_fun(frame_shape)

    #frame = cv2.flip(frame,0)
    # out.write(cv_image)

    # img.append(cv_image)
    rospy.sleep(0.1)
    c = 1+c

    print "The number amount of captured frames: ", c
    if (c == frame_num):
        print "Video end "
        rospy.signal_shutdown("Video end")


rospy.init_node('video_handler', anonymous=True)
bridge = CvBridge()
# writer = skvideo.io.FFmpegWriter("outputvideo.mp4")
rospy.on_shutdown(_shutdown)

rospy.Subscriber("/kinect/depth/image_raw",
                         Image, lambda img: _Img_callback(img, bridge, writer), queue_size=2)
rospy.wait_for_message("/kinect/depth/image_raw", Image)

rospy.spin()
