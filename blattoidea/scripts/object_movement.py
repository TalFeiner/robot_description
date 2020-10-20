#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Point,Twist, Pose
from tf2_geometry_msgs import PoseStamped
from shapely import geometry
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

global coords, model_state
coords = []

def model_state_callback(msg):
    global model_state
    model_state = msg

def poly_callback(msg):
    global coords
    coords = []
    # print "point", msg.polygon.points
    for ii in range (len(msg.polygon.points)):
        poly_coords = (msg.polygon.points[ii].x, msg.polygon.points[ii].y)
        coords.append(poly_coords)

def rotation_z_axis(v, theta):
    R = np.array(([np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0] ,[0, 0, 1]))
    return R.dot(v.T)

def spawn_poly(gazebo_model_srv, obj, reference, mean_vel = [0, -1], cov_vel = [[0.3, 0], [0, 3]]):
    global coords, model_state
    blattoidea_idx = model_state.name.index ("blattoidea")
    blattoidea_orientation = np.array((model_state.pose[blattoidea_idx].orientation.x,model_state.pose[blattoidea_idx].orientation.y, model_state.pose[blattoidea_idx].orientation.z, model_state.pose[blattoidea_idx].orientation.w))
    
    x = float(np.random.uniform(-2, 2, 1))
    y = float(np.random.uniform(3.5, 5.9, 1))
    vel = np.random.multivariate_normal([0, -4], [[2, 0], [0, 1]], (1)).reshape(2)
    
    model_vel = Twist()
    model_vel.linear.x = vel[0]
    model_vel.linear.y = vel[1]
    model_vel.linear.z = 0

    model_pose = Pose()
    model_pose.position.x = x
    model_pose.position.y = y
    model_pose.position.z = model_state.pose[model_state.name.index(obj)].position.z
    model_pose.orientation.x = blattoidea_orientation[0]
    model_pose.orientation.y = blattoidea_orientation[1]
    model_pose.orientation.z = blattoidea_orientation[2]
    model_pose.orientation.w = blattoidea_orientation[3]
    
    model = ModelState()
    model.model_name = obj
    model.twist = model_vel
    model.pose = model_pose
    model.reference_frame = reference
    
    gazebo_model_srv(model)
    
def point_poly(tfBuffer, obj, frame, poly, bool):

    global model_state
    poly_coords = []
    if bool:
        for ii in xrange (len(poly)):
            pose = PoseStamped()
            pose.pose.position.x = poly[ii][0]
            pose.pose.position.y = poly[ii][1]
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header.frame_id = frame
            pose.header.stamp = rospy.Time(0)
            try:
                p = tfBuffer.transform(pose, 'world')
                x, y = p.pose.position.x, p.pose.position.y
            except tf2_ros.TransformException as e:
                print ("some tf2 exception happened", e)
            poly_coords.append((x, y))
    else:
        poly_coords = poly

    polygon = geometry.Polygon(poly_coords)
    inside_list = []
    for ii in xrange(len(obj)):
        idx = model_state.name.index(obj[ii])
        point = geometry.Point(model_state.pose[idx].position.x, model_state.pose[idx].position.y)
        within = point.within(polygon)
        inside_list.append(within)
    return inside_list

def camera_poly(frame):
    global coords
    poly = PolygonStamped()
    poly.polygon.points = []

    for ii in xrange(len(coords)):
        poly_point = Point()
        poly_point.x = coords[ii][0]
        poly_point.y = coords[ii][1]
        poly_point.z = 0.0
        poly.polygon.points.append(poly_point)

    poly.header.frame_id = frame
    poly.header.stamp = rospy.Time.now()
    return poly

# def is_crashed(gazebo_model,triangle_coords):

    


def main():
    global coords, model_state
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    poly_pub = rospy.Publisher('/poly_pub', PolygonStamped, queue_size=2)
    gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

    x1 = 4
    y1 = 1
    x2 = -4
    y2 = 6
    
    poly_coords = [(0.0, y1), (x1, y1), (x1, y2), (x2, y2), (x2, y1), (0.0, y1)]
    triangle = [(0.0,0.0),(0.2,0.0),(0.2,0.8),(-0.2,0.8),(-0.2,0),(0.0,0.0)]

    
    coords = poly_coords
    poly = camera_poly("base_footprint")
    poly_pub.publish(poly)
    

    r = rospy.Rate(2)
    model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    tf.TransformListener().waitForTransform("world", "base_footprint", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        poly = camera_poly("base_footprint")
        poly_pub.publish(poly)

        obj_poly = []
        #obj_square = []
        obj_poly.append('sphere_red')
        
        inside_poly = point_poly(tfBuffer, obj_poly, "base_footprint", poly_coords, True)
        is_crashed = point_poly(tfBuffer, obj_poly, "base_footprint", triangle, True)
        #inside_square = point_poly(tfBuffer, obj_square, "base_footprint", square_coords, False)

        for ii in range (len(obj_poly)):
            if not inside_poly[ii]:
                try:
                    spawn_poly(gazebo_model_srv, obj = obj_poly[ii], reference = "base_footprint")
                except:
                    rospy.logwarn("Spawn model " + obj_poly[ii] + " Failed!!")
                    pass
        # print "inside", inside
        if is_crashed[0]:
            print ('Catched!')
            try:
                spawn_poly(gazebo_model_srv, obj = obj_poly[ii], reference = "base_footprint")
            except:
                rospy.logwarn("Spawn model " + obj_poly[ii] + " Failed!!")
                pass
            
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("advertise_sphere", anonymous=True)
    rospy.wait_for_service("gazebo/set_model_state")
    # rospy.Subscriber('/poly_pub', PolygonStamped, poly_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
    
    main()
    