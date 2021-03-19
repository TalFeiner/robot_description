#!/usr/bin/env python
import rosbag
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates




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
    return

def Color_and_Velocity_finder(  imgg , Red_velocity=[0,0,0] , Blue_velocity=[0,0,0],
                                Green_velocity=[0,0,0]    , Yellow_velocity=[0,0,0],
                                Orange_velocity=[0,0,0]   , Purple_velocity=[0,0,0],
                                Turquoise_velocity=[0,0,0] ,  Gold_velocity=[0,0,0]):
    
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
    
    # Orange color:
    low_orange = np.array([11,106,92])
    high_orange = np.array([15,255,189])
    Orange = cv2.inRange(img_HSV,low_orange,high_orange)
    Orange_Vel_mat = _Matrix_Velocity(Orange,Orange_velocity)
    
    # Purple color:
    low_purple = np.array([139,96,91])
    high_purple = np.array([168,255,161])
    Purple = cv2.inRange(img_HSV,low_purple,high_purple)
    Purple_Vel_mat = _Matrix_Velocity(Purple,Purple_velocity)
    
    # Turquoise color:
    low_turquoise = np.array([88,98,103])
    high_turquoise = np.array([93,255,255])
    Turquoise = cv2.inRange(img_HSV,low_turquoise,high_turquoise)
    Turquoise_Vel_mat = _Matrix_Velocity(Turquoise,Turquoise_velocity)
    
    # Gold color:
    low_gold = np.array([14,171,46])
    high_gold = np.array([29,255,111])
    Gold = cv2.inRange(img_HSV,low_gold,high_gold)
    Gold_Vel_mat = _Matrix_Velocity(Gold,Gold_velocity)


    _img_finding_shape(Red,imgg,vel = Red_velocity,color = "Red")
    _img_finding_shape(Blue,imgg,vel = Blue_velocity,color = "Blue")
    _img_finding_shape(Green,imgg,vel = Green_velocity,color = "Green")
    _img_finding_shape(Yellow,imgg,vel = Yellow_velocity,color = "Yellow")
    _img_finding_shape(Orange,imgg,vel = Orange_velocity,color = "Orange")
    _img_finding_shape(Purple,imgg,vel = Purple_velocity,color = "Purple")
    _img_finding_shape(Turquoise,imgg,vel = Turquoise_velocity,color = "Turquoise")
    _img_finding_shape(Gold,imgg,vel = Gold_velocity,color = "Gold")
    
    
    Velocity_matrix = Red_Vel_mat    + Blue_Vel_mat   +\
                      Green_Vel_mat  + Yellow_Vel_mat +\
                      Orange_Vel_mat + Purple_Vel_mat +\
                      Turquoise_Vel_mat + Gold_Vel_mat

    
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



def stackImages(scale,imgArray):
    '''Taken from Murtaza's Workshop - Robotics and AI. Not mine!!'''
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver
