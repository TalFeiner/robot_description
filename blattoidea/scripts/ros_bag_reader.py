#!/usr/bin/env python

import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
flag = True
# img = []
bridge = CvBridge()
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
bag = rosbag.Bag('/home/melodic/Documents/test_2020-10-01-15-08-20.bag')
for topic, msg, t in bag.read_messages(topics=['/kinect/depth/image_raw']):  #  , '/kinect/color/image_raw/compressedDepth' 
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    except CvBridgeError as e:
        print(e)
    # np_arr = np.array(cv_image).astype(np.float)
    cv_image = np.array(cv_image, dtype=np.float)
    cv_image = cv2.normalize(cv_image, cv_image, 0, 255, cv2.NORM_MINMAX)
    cv_image = np.round(cv_image).astype(np.uint8)
    #frame = cv2.imdecode(np_arr, cv2.CV_16UC1)
    if flag:
        flag = False
        shape = np.shape(cv_image)
        print "cv_image", cv_image
        is_color = False
        out = cv2.VideoWriter('video.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), is_color) 
    out.write(cv_image)
    msg_img = np.array(cv_image).astype(np.uint8)
    # img.append(msg_img)
bag.close()

# import skvideo.io



# skvideo.io.vwrite("outputvideo.mp4", img)
