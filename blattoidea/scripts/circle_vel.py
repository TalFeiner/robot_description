#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist, Pose

rospy.init_node('gazebo_circle', anonymous=True)
rospy.wait_for_service("gazebo/set_model_state")
gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

while not rospy.is_shutdown():
    gazebo_model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    model_vel = Twist()
    x = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].position.x
    y = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].position.y
    angle = np.arctan2(y,x)
    print "angle ", angle

    r = 5
    vel_x = -np.sin(angle) * 1.4
    vel_y = np.cos(angle) * 1.4

    model_vel.linear.x = vel_x
    model_vel.linear.y = vel_y
    model_vel.linear.z = 0

    model_pose = Pose()
    model_pose.position.x = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].position.x
    model_pose.position.y = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].position.y
    model_pose.position.z = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].position.z
    model_pose.orientation.x = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].orientation.x
    model_pose.orientation.y = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].orientation.y
    model_pose.orientation.z = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].orientation.z
    model_pose.orientation.w = gazebo_model_state.pose[gazebo_model_state.name.index("sphere_red")].orientation.w

    model = ModelState()
    model.model_name = "sphere_red"
    model.twist = model_vel
    model.pose = model_pose

    gazebo_model_srv(model)

