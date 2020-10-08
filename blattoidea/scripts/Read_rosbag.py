#!/usr/bin/env python
import rosbag
import cv2
import numpy as np
import time

from Vital_functions import Velocity_from_gazebo, Color_and_Velocity_finder , stackImages

from scipy.io import savemat

from cv_bridge import CvBridge, CvBridgeError


bag = rosbag.Bag("my_bag.bag")
flag = True
cap = cv2.VideoCapture('color.avi')
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
time_step = []
gazebo_name = '/gazebo/model_states'
i = 0
#models_list = ["sphere_blue","sphere_green","sphere_red","sphere_yellow"]
#model_list_2 = ["cylinder_purple", "cylinder_orange", "cylinder_turquoise", "cylinder_gold"]
for topic , msg , t in bag.read_messages(topics = [gazebo_name]):
    
    ret , frame = cap.read()

    vel_R = Velocity_from_gazebo(msg , 'sphere_red' )
    vel_B = Velocity_from_gazebo(msg , 'sphere_blue' )
    vel_G = Velocity_from_gazebo(msg , 'sphere_green' )
    vel_Y = Velocity_from_gazebo(msg , 'sphere_yellow' )
    vel_P = Velocity_from_gazebo(msg , 'cylinder_purple' )
    vel_O = Velocity_from_gazebo(msg , 'cylinder_orange' )
    vel_T = Velocity_from_gazebo(msg , 'cylinder_turquoise' )
    vel_G = Velocity_from_gazebo(msg , 'cylinder_gold' )

    

    Img = np.array(frame , dtype = np.uint8)

    Img , Matrix = Color_and_Velocity_finder(Img,Red_velocity=vel_R,Blue_velocity=vel_B,
                                            Green_velocity=vel_G,Yellow_velocity=vel_Y,
                                            Orange_velocity=vel_G,Purple_velocity=vel_P,
                                            Turquoise_velocity=vel_T,Gold_velocity=vel_G)
    Matrix = np.array(Matrix,dtype=np.uint8)
    
    if i == 0:
        shape = np.shape(Matrix)
        out_c = cv2.VideoWriter('Velocity.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), True)
        t_first = t
        
    step = t.secs-t_first.secs + (10**-9)*(t.nsecs-t_first.nsecs)
    time_step.append(step)
    out_c.write(Matrix)
    i = i + 1

    comb_img = stackImages(1,[Img,Matrix])
    #cv2.imshow("velocity_image",Matrix)
    #cv2.imshow("Image",Img)
    imshow("color_and_velocity",comb_img)
    cv2.waitKey(1)
    print(i)

time_step = np.array(time_step,dtype=np.float16)
savemat("Second_batch",{'Time_Stamp' : time_step})
print np.round(step,2) , ' seconds recording'

cap.release()
cv2.destroyAllWindows()
out_c.release()
bag.close()





