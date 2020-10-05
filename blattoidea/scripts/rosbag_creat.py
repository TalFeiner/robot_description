#!/usr/bin/env python
import rosbag
import rospy
import cv2
import numpy as np

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
bag = rosbag.Bag('my_bag.bag','w')
while not rospy.is_shutdown():

    Img_color = color_image
    Img_depth = depth_image
    Model = model

    bag.write("kinect/color/image_raw",Img_color)
    bag.write("kinect/depth/image_raw",Img_depth)
    bag.write("/gazebo/model_states",Model)


bag.close()
print "Done creating"