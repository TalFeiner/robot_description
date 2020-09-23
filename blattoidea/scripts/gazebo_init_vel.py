#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('gazebo_link_pose', anonymous=True)
    rospy.wait_for_service("gazebo/set_model_state")
    gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

    model_vel = Twist()
    model_vel.linear.x = 1

    model = ModelState()
    model.model_name = "unit_sphere"
    model.reference_frame = "world"
    model.twist = model_vel

    gazebo_model_srv(model)
