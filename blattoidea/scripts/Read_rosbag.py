#!/usr/bin/env python
import rosbag
import cv2
import numpy as np
import time
from scipy.io import savemat

from cv_bridge import CvBridge, CvBridgeError

def _img_finding_shape(imgg,origin_img,vel = [0,0,0],font = cv2.FONT_HERSHEY_COMPLEX , color = "Ball"):
    
    red_vel_round = np.round(np.array(vel),2)
    vel_text = str(red_vel_round)
    '''Gray image input and original. Center of shape output'''
    ret, threshold = cv2.threshold(imgg, 240, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    iteration = -1
    x=-1
    y=-1
    for iteration,cnt in enumerate(contours):
        # Area in pixels:
        area = cv2.contourArea(cnt)
        areaMin = 550
        # Only if the ball is big enough:
        if area > areaMin:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
            x , y , w, h = cv2.boundingRect(approx)
            cv2.putText(origin_img, color , (x+4*w/10, y+h/2), font, 0.5, (0,0,0))
            cv2.putText(origin_img, vel_text , (x+4*w/10, y+h/2+12), font, 0.3, (0,0,0))
        else: continue
    if color == 'Red' and iteration == -1:
        print "Problem"
    return x,y

def Color_and_Velocity_finder(imgg , Red_velocity=[0,0,0] , Blue_velocity=[0,0,0],
                                Green_velocity=[0,0,0] , Yellow_velocity=[0,0,0] ):
    
    img_HSV = cv2.cvtColor(imgg,cv2.COLOR_BGR2HSV)
    
    # Red color:
    low_red = np.array([0,120,100])
    high_red = np.array([1,255,255])
    Red = cv2.inRange(img_HSV,low_red,high_red)
    Red_Vel_mat = _Matrix_Velocity(Red,Red_velocity)
    
    # Blue color:
    low_blue = np.array([120,251,101])
    high_blue = np.array([121,255,132])
    Blue = cv2.inRange(img_HSV,low_blue,high_blue)
    Blue_Vel_mat = _Matrix_Velocity(Blue,Blue_velocity)
    
    # Green color:
    low_green = np.array([59,251,101])
    high_green = np.array([60,255,132])
    Green = cv2.inRange(img_HSV,low_green,high_green)
    Green_Vel_mat = _Matrix_Velocity(Green,Green_velocity)
    
    # Yellow color:
    low_yellow = np.array([29,251,101])
    high_yellow = np.array([30,255,132])
    Yellow = cv2.inRange(img_HSV,low_yellow,high_yellow)
    Yellow_Vel_mat = _Matrix_Velocity(Yellow,Yellow_velocity)
    
    x_red,y_red = _img_finding_shape(Red,imgg,vel = Red_velocity,color = "Red")
    x_red,y_red = _img_finding_shape(Blue,imgg,color = "Blue")
    x_red,y_red = _img_finding_shape(Green,imgg,color = "Green")
    x_red,y_red = _img_finding_shape(Yellow,imgg,color = "Yellow")
    
    Velocity_matrix = Red_Vel_mat + Blue_Vel_mat +\
    Green_Vel_mat  + Yellow_Vel_mat
    
    return imgg , Velocity_matrix
    
def _Matrix_Velocity(img,velocity):
    '''Takes gray image and the sphere velocity. Return sphere matrix velocity'''
    
    img_matrix = img.astype(int)
    img_matrix[img_matrix>0] = 1
    
    img_matrix = np.dstack((img_matrix,img_matrix,img_matrix))
    vel_vector = np.full((img.shape[0],img.shape[1],3),velocity)
    velocity_matrix = img_matrix * vel_vector
    
    return velocity_matrix



global vel
bridge = CvBridge()
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
bag = rosbag.Bag('my_bag.bag')

'''
flag = True
for topic, msg, t in bag.read_messages(topics=['kinect/depth/image_raw']):  #  , '/kinect/color/image_raw/compressedDepth' 
    try:
        # encoding of simulated depth image: 32FC1, encoding of real depth image: 16UC1
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    except CvBridgeError as e:
        print(e)
    cv_image = cv2.resize(cv_image,(400,400),interpolation = cv2.INTER_AREA)
    cv_image = np.array(cv_image, dtype=np.float)
    cv_image = cv2.normalize(cv_image, cv_image, 0, 255, cv2.NORM_MINMAX)
    cv_image = np.round(cv_image).astype(np.uint8)
    
    if flag:
        flag = False
        shape = np.shape(cv_image)
        print "Start making video"
        is_color = False
        out = cv2.VideoWriter('video.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), is_color) 
    out.write(cv_image)
if not flag:
    print "Made video"
'''

flag = True
for topic , msg , t in bag.read_messages(topics = ['kinect/color/image_raw']):
    try:
        # encoding of simulated depth image: 32FC1, encoding of real depth image: 16UC1
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    
    cv_image = cv2.resize(cv_image,(400,400),interpolation = cv2.INTER_AREA)
    cv_image = np.expand_dims(cv_image,axis=3)

    if flag:
        flag = False
        img_stream = cv_image
        print img_stream.shape
    else:
        img_stream = np.concatenate([img_stream,cv_image],axis = 3)    
    

time_step = []
gazebo_name = '/gazebo/model_states'
i = 0
for topic , msg , t in bag.read_messages(topics = [gazebo_name]):
    colors = msg.name
    index = colors.index("unit_sphere")
    vel = np.array([msg.twist[index].linear.x,
                    msg.twist[index].linear.y,
                    msg.twist[index].linear.z])
    Img = np.array(img_stream[:,:,:,i] , dtype = np.uint8)
    Img , Matrix = Color_and_Velocity_finder(Img,Red_velocity=vel)
    
    pic_matrix = np.abs(Matrix)
    pic_matrix = pic_matrix * 20

    Matrix = np.expand_dims(Matrix,axis = 3)
    if i == 0:
        t_first = t
        Matrix_stream = Matrix
    else:
        Matrix_stream = np.concatenate([Matrix_stream,Matrix],axis=3)
    #Matrix_stream[:,:,:,i] = t.secs-t_first.secs + (10**-9)*(t.nsecs-t_first.nsecs)
    step = t.secs-t_first.secs + (10**-9)*(t.nsecs-t_first.nsecs)
    time_step.append(step)
    
    i = i + 1
    cv2.imshow("velocity_image",pic_matrix)
    cv2.imshow("Image",Img)
    cv2.waitKey(1)
    print(i)

time_step = np.array(time_step)
savemat("Second_batch",{'pixel_Velocity' : Matrix_stream ,'Time_Stamp' : time_step})

cv2.destroyAllWindows()

bag.close()





