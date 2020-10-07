#!/usr/bin/env python
import rosbag
import cv2
import numpy as np
import time
from scipy.io import savemat

from cv_bridge import CvBridge, CvBridgeError


bag = rosbag.Bag("my_bag.bag")

def _img_finding_shape(imgg,origin_img,vel = [0,0,0],font = cv2.FONT_HERSHEY_COMPLEX , color = "Ball"):
    
    vel = Velocity_UnNormalize(vel)
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
    low_blue = np.array([108,101,0])
    high_blue = np.array([123,255,255])
    Blue = cv2.inRange(img_HSV,low_blue,high_blue)
    Blue_Vel_mat = _Matrix_Velocity(Blue,Blue_velocity)
    
    # Green color:
    low_green = np.array([54,101,0])
    high_green = np.array([61,255,255])
    Green = cv2.inRange(img_HSV,low_green,high_green)
    Green_Vel_mat = _Matrix_Velocity(Green,Green_velocity)
    
    # Yellow color:
    low_yellow = np.array([28,101,0])
    high_yellow = np.array([34,255,255])
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

def Velocity_Normalize(Velocity):
    '''Input - Velocity (-5,5). Output - normalized Velocity (0,250) '''
    Velocity[Velocity>0.01] = np.array(25 * Velocity[Velocity>0.01] + 125)
    Velocity[Velocity<-0.01] = np.array(-25 * Velocity[Velocity<-0.01])
    return Velocity


def Velocity_UnNormalize(Velocity):
    '''Input - Velocity (0,250). Output - unnormalized Velocity (-5,5)'''
    Velocity = np.array(Velocity,dtype=np.float32)
    Velocity[Velocity>125] = (Velocity[Velocity>125] - 125)/25
    Velocity[Velocity<125] =  Velocity[Velocity<125]/-25
    return Velocity

def Velocity_from_gazebo(msg , unit_name):
    colors = msg.name
    index = colors.index(unit_name)
    vel = np.array([-msg.twist[index].linear.y,
                    msg.twist[index].linear.z,
                    -msg.twist[index].linear.x])
    vel = Velocity_Normalize(vel)
    return vel


global vel
flag = True
cap = cv2.VideoCapture('color.avi')
fourcc = cv2.VideoWriter_fourcc(*'DIVX')

for i in range(1000):
    ret , frame = cap.read()
    frame = np.expand_dims(np.array(frame,dtype=np.uint8),axis=3)
    if flag:
        flag = False
        img_stream = frame
    else:
        print img_stream.shape
        print frame.shape
        img_stream = np.concatenate([img_stream,frame],axis = 3)  
 
cap.release()

time_step = []
gazebo_name = '/gazebo/model_states'
i = 0
for topic , msg , t in bag.read_messages(topics = [gazebo_name]):
    
    vel_R = Velocity_from_gazebo(msg , 'sphere_red' )
    vel_B = Velocity_from_gazebo(msg , 'sphere_blue' )
    vel_G = Velocity_from_gazebo(msg , 'sphere_green' )
    vel_Y = Velocity_from_gazebo(msg , 'sphere_yellow' )

    Img = np.array(img_stream[:,:,:,i] , dtype = np.uint8)

    Img , Matrix = Color_and_Velocity_finder(Img,Red_velocity=vel_R,Blue_velocity=vel_B,
                                            Green_velocity=vel_G,Yellow_velocity=vel_Y)
    Matrix = np.array(Matrix,dtype=np.uint8)
    
    if i == 0:
        shape = np.shape(Matrix)
        out_c = cv2.VideoWriter('Velocity.avi', fourcc, 20.0, (int(shape[1]), int(shape[0])), True)
        #t_first = t
        
    #step = t.secs-t_first.secs + (10**-9)*(t.nsecs-t_first.nsecs)
    #time_step.append(step)
    out_c.write(Matrix)
    i = i + 1
    cv2.imshow("velocity_image",Matrix)
    cv2.imshow("Image",Img)
    cv2.waitKey(1)
    print(i)

#time_step = np.array(time_step,dtype=np.float16)
#savemat("Second_batch",{'Time_Stamp' : time_step})

cv2.destroyAllWindows()
out_c.release()
bag.close()





