#!/usr/bin/env python

import rospy
import numpy as np
# import tf2_ros
from geometry_msgs.msg import PolygonStamped, Point,Twist, Pose
from shapely import geometry
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState

def obj_place(model_state, obj, inside, gazebo_model_srv, coords):
    for ii in range (len(obj)):
        if not inside[ii]:
            spawn(ii ,model_state, coords, gazebo_model_srv = gazebo_model_srv, obj = obj[ii], reference = "camera_link")

def spawn(ii, model_state, coords, gazebo_model_srv, obj, reference, mean_vel = [0, 0], cov_vel = [[6, 0], [0, 6]]):
    vel = np.random.multivariate_normal(mean_vel, cov_vel, (1)).reshape(2)
    for ii in range(len(vel)):
        if vel[ii] is 0.0:
            vel[ii] = 1
    model_vel = Twist()
    model_vel.linear.x = vel[0]
    model_vel.linear.y = vel[1]
    model_vel.linear.z = 0
    
    x = float(np.random.uniform(coords[0][0], coords[2][0], 1))
    y = float(np.random.normal(0,0.5,1) + x * np.sqrt(3))
    model_pose = Pose()
    model_pose.position.x = x
    model_pose.position.y = y
    model_pose.position.z = model_state.pose[ii].position.z
    
    model = ModelState()
    model.model_name = obj
    model.twist = model_vel
    model.pose = model_pose
    model.reference_frame = reference
    
    gazebo_model_srv(model)
    
def point_poly(coords, model_state, obj):
    polygon = geometry.Polygon(coords)
    inside_list = []
    for ii in xrange(len(obj)):
        idx = model_state.name.index(obj[ii])
        point = geometry.Point(model_state.pose[idx].position.x, model_state.pose[idx].position.y)
        within = point.within(polygon)
        inside_list.append(within)
    return inside_list

def camera_poly(coords, frame):
    poly = PolygonStamped()
    poly.polygon.points = []

    poly_point = Point()
    poly_point.x = coords[0][0]
    poly_point.y = coords[0][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = coords[1][0]
    poly_point.y = coords[1][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = coords[2][0]
    poly_point.y = coords[2][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = coords[3][0]
    poly_point.y = coords[3][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)
    
    poly_point = Point()
    poly_point.x = coords[4][0]
    poly_point.y = coords[4][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = coords[5][0]
    poly_point.y = coords[5][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly.header.frame_id = frame
    poly.header.stamp = rospy.Time.now()
    return poly

def main():
    rospy.init_node("advertise_polygon", anonymous=True)
    rospy.wait_for_service("gazebo/set_model_state")
    poly_pub = rospy.Publisher('poly_pub', PolygonStamped, queue_size=2)
    gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    
    coords = [(1.4, 0.0), (1.4, 3), (6, 11), (6, -11), (1.4, -3), (1.4, 0.0)]
    poly = camera_poly(coords, "camera_link")
    
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        obj = ["unit_sphere"]
        poly_pub.publish(poly)
        inside = point_poly(coords, model_state, obj)
        obj_place(model_state, obj = obj, inside = inside, gazebo_model_srv = gazebo_model_srv, coords = coords)
        print "inside", inside

        r.sleep()

if __name__ == "__main__":
    main()