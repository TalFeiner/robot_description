#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist, Pose

def rotation_z_axis(v, theta):
    R = np.array(([np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0] ,[0, 0, 1]))
    return R.dot(v.T)

class obj_pose():

    def __init__ (self, robot_model = "blattoidea", model_to_throw = "unit_sphere", min_dist_between_models = 1.6, mean = [4, 0, 0], cov = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        rospy.init_node('gazebo_link_pose', anonymous=True)
        rospy.wait_for_service("gazebo/set_model_state")
        self.robot_model = robot_model
        self.model_to_throw = model_to_throw
        self.min_dist_between_models = min_dist_between_models
        self.mean = mean
        self.cov = cov


    def check_pose(self):
        while not rospy.is_shutdown():
            model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)
            idx = model_state.name.index (self.model_to_throw)
            blattoidea_idx = model_state.name.index (self.robot_model)
            position =  np.array([model_state.pose[idx].position.x, model_state.pose[idx].position.y, model_state.pose[idx].position.z])
            blattoidea_position = np.array((model_state.pose[blattoidea_idx].position.x, model_state.pose[blattoidea_idx].position.y, model_state.pose[blattoidea_idx].position.z))
            r = np.linalg.norm(np.array([blattoidea_position[0], blattoidea_position[1]]) - np.array([position[0], position[1]]))
            if r < self.min_dist_between_models:
                break

        self.gazebo_pose()

    def gazebo_pose(self):
        gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        gazebo_model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)

        model_to_throw_idx = gazebo_model_state.name.index (self.model_to_throw)
        blattoidea_idx = gazebo_model_state.name.index (self.robot_model)
        blattoidea_position = np.array((gazebo_model_state.pose[blattoidea_idx].position.x, gazebo_model_state.pose[blattoidea_idx].position.y, gazebo_model_state.pose[blattoidea_idx].position.z))
        blattoidea_orientation = np.array((gazebo_model_state.pose[blattoidea_idx].orientation.x, gazebo_model_state.pose[blattoidea_idx].orientation.y, gazebo_model_state.pose[blattoidea_idx].orientation.z, gazebo_model_state.pose[blattoidea_idx].orientation.w))
        blattoidea_euler = np.array(euler_from_quaternion(blattoidea_orientation))

        position = (rotation_z_axis(np.random.multivariate_normal(self.mean, self.cov, (1)), blattoidea_euler[2]).T + blattoidea_position).reshape((3))
        vel = (position - blattoidea_position) * -1.2

        print "vel", vel
        model_vel = Twist()
        model_vel.linear.x = vel[0]
        model_vel.linear.y = vel[1]
        model_vel.linear.z = 0

        model_pose = Pose()
        model_pose.position.x = position[0]
        model_pose.position.y = position[1]
        model_pose.position.z = gazebo_model_state.pose[model_to_throw_idx].position.z

        model = ModelState()
        model.model_name = self.model_to_throw
        model.twist = model_vel
        model.pose = model_pose

        gazebo_model_srv(model)
        self.check_pose()

if __name__ == '__main__':
    pose = obj_pose()   
    pose.gazebo_pose()
