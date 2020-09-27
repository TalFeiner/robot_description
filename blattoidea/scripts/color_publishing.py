#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

def _img_finding_shape(img,origin_img,font = cv2.FONT_HERSHEY_COMPLEX , color = "Ball"):
    
    '''Gray image input and original. return center of shape output'''
    ret, threshold = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for iteration,cnt in enumerate(contours):
        # Area in pixels:
        area = cv2.contourArea(cnt)
        areaMin = 550
        x=[]
        y=[]
        # Only if the ball is big enough:
        if area > areaMin:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
            x , y , w, h = cv2.boundingRect(approx)
            cv2.putText(origin_img, color , (x+4*w/10, y+h/2), font, 0.5, (0,0,0))
        else: continue
    return x,y

def red_filter(img):

    '''Raw image input. return [compressed image, center of red shape vector of (x,y)]'''

    img_comp = cv2.resize(img,(300,300),interpolation = cv2.INTER_AREA)
    #img_comp = img
    img_HSV = cv2.cvtColor(img_comp,cv2.COLOR_BGR2HSV)
    
    
    # Red color:
    low_red = np.array([0,10,0])
    high_red = np.array([50,255,255])
    
    Red = cv2.inRange(img_HSV,low_red,high_red)
    x_red,y_red = _img_finding_shape(Red,img_comp,color = "Red")
    
    return img_comp , x_red , y_red


global img_data , bridge

def call_back_camera(ros_img):
    
    global img_data, bridge
    try: 
        img_data = bridge.imgmsg_to_cv2(ros_img,"bgr8")
    except CvBridgeError as e:
        print (e)
    
bridge = CvBridge()
rospy.init_node("ball_color_naming_node")
subs = rospy.Subscriber("kinect/color/image_raw",Image, call_back_camera)
rospy.wait_for_message("kinect/color/image_raw",Image)


while not rospy.is_shutdown():
    
    img = img_data
    img_comp, x_red , y_red = red_filter(img)
    cv2.imshow("Simulation",img_comp)
    cv2.waitKey(1)

cv2.destroyAllWindows()