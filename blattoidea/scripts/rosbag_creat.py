#!/usr/bin/env python
import rosbag
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates


global color_image,model,depth_image

def call_back_camera(ros_img):
    global color_image 
    color_image = ros_img
  
def Model_Callback(data):
    global model
    model = data
    
def _Img_callback(ros_data):
    global depth_image
    depth_image = ros_data

rospy.init_node("rosbag_node")
subs = rospy.Subscriber("kinect/color/image_raw",Image, call_back_camera,queue_size=1)
rospy.wait_for_message("kinect/color/image_raw",Image)
subs_model = rospy.Subscriber("/gazebo/model_states",ModelStates,Model_Callback,queue_size=1)
rospy.wait_for_message("/gazebo/model_states",ModelStates)
rospy.Subscriber("/kinect/depth/image_raw",
                         Image, _Img_callback,queue_size=1)
rospy.wait_for_message("/kinect/depth/image_raw", Image)

print "Create"
bridge = CvBridge()
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
NumberOfSamples = 5000

bag = rosbag.Bag('my_bag.bag','w')
flag = True
i = 0
while not rospy.is_shutdown():


    Model = model
    bag.write("/gazebo/model_states",Model)

    Img_color = color_image
    Img_depth = depth_image
    
    try:
        # encoding of simulated depth image: 32FC1, encoding of real depth image: 16UC1
        Img_d = bridge.imgmsg_to_cv2(Img_depth, desired_encoding="32FC1")
        Img_c = bridge.imgmsg_to_cv2(Img_color, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    
    Img_d = cv2.resize(Img_d,(256,256),interpolation = cv2.INTER_AREA)
    Img_d = np.array(Img_d, dtype=np.float)
    Img_d = cv2.normalize(Img_d, Img_d, 0, 255, cv2.NORM_MINMAX)
    Img_d = np.round(Img_d).astype(np.uint8)

    Img_c = cv2.resize(Img_c,(256,256),interpolation = cv2.INTER_AREA)
    Img_c = np.array(Img_c, dtype=np.float)
    Img_c = cv2.normalize(Img_c, Img_c, 0, 255, cv2.NORM_MINMAX)
    Img_c = np.round(Img_c).astype(np.uint8)
    
    if flag:
        flag = False
        shape = np.shape(Img_d)
        print "Start making video"
        is_color = False
        out_d = cv2.VideoWriter('depth.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), is_color)
        out_c = cv2.VideoWriter('color.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), True)
    out_d.write(Img_d)
    out_c.write(Img_c)
    

    i = i + 1
    print i , " frames have been taken"
    if i == NumberOfSamples:
        break

bag.close()
out_d.release()
out_d.release()
print "Done!!"