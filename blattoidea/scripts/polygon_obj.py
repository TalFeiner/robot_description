#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PolygonStamped, Point,Twist, Pose
from shapely import geometry
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global coords
coords = []

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

def obj_place(model_state, blattoidea_idx, obj, inside, gazebo_model_srv):
    for ii in range (len(obj)):
        if not inside[ii]:
            try:
                spawn(ii, blattoidea_idx, model_state, gazebo_model_srv = gazebo_model_srv, obj = obj[ii], reference = "base_linke")
            except:
                pass

def spawn(ii, blattoidea_idx, model_state, gazebo_model_srv, obj, reference, mean_vel = [0, 0], cov_vel = [[6, 0], [0, 6]]):
    global coords
    blattoidea_position = np.array((model_state.pose[blattoidea_idx].position.x, model_state.pose[blattoidea_idx].position.y, model_state.pose[blattoidea_idx].position.z))
    blattoidea_orientation = np.array((model_state.pose[blattoidea_idx].orientation.x,model_state.pose[blattoidea_idx].orientation.y, model_state.pose[blattoidea_idx].orientation.z, model_state.pose[blattoidea_idx].orientation.w))
    blattoidea_euler = np.array(euler_from_quaternion(blattoidea_orientation))
    # vel = np.random.multivariate_normal(mean_vel, cov_vel, (1)).reshape(2)
    # for ii in range(len(vel)):
    #     if vel[ii] is 0.0:
    #         vel = np.random.multivariate_normal([3, 3], [[2, 0], [0, 2]], (1)).reshape(2)
    # model_vel = Twist()
    # model_vel.linear.x = vel[0]
    # model_vel.linear.y = vel[1]
    # model_vel.linear.z = 0
    
    # rand = int(np.random.randint(3, size=1))
    # print "coords", coords
    rand = int(np.random.randint(3, size=1))
    if rand is 0:
        x = float(np.random.uniform(coords[0][0], coords[2][0], 1))
        if (x >= 0 and coords[0][0] >= 0) or (x < 0 and coords[0][0] < 0):
            y = float((np.sqrt(3) * x - coords[0][0]) * -1)
            vel = np.random.multivariate_normal([0.5, 4], [[1, 0], [0, 1]], (1)).reshape(2)
        elif (x < 0 and coords[0][0] > 0) or (x > 0 and coords[0][0] < 0):
            y = float((np.sqrt(3) * x + coords[0][0]) * -1)
            vel = -1 * np.random.multivariate_normal([0.5, 4], [[1, 0], [0, 1]], (1)).reshape(2)
        # position = np.array((x, y, 0))
        # x, y, __ = (rotation_z_axis(position, blattoidea_euler[2]).T + blattoidea_position).reshape((3))
        # vel[0] = 0
        # for ii in range(len(vel)):
        #     if vel[ii] is 0.0:
        #         vel = np.random.multivariate_normal([3, 3], [[2, 0], [0, 2]], (1)).reshape(2)
    elif rand is 1:
        x = float(np.random.uniform(coords[0][0], coords[2][0], 1))
        if (x >= 0 and coords[0][0] >= 0) or (x < 0 and coords[0][0] < 0):
            y = float(np.sqrt(3) * x - coords[0][0])
            vel = np.random.multivariate_normal([0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)
        elif (x < 0 and coords[0][0] > 0) or (x > 0 and coords[0][0] < 0):
            y = float(np.sqrt(3) * x + coords[0][0])
            vel = -1 * np.random.multivariate_normal([0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)
        # position = np.array((x, y, 0))
        # x, y, __ = (rotation_z_axis(position, blattoidea_euler[2]).T + blattoidea_position).reshape((3))
        # vel[0] = 0
        # for ii in range(len(vel)):
        #     if vel[ii] is 0.0:
        #         vel = np.random.multivariate_normal([3, 3], [[2, 0], [0, 2]], (1)).reshape(2)
    elif rand is 2:
        y = float(np.random.uniform(coords[1][1], coords[2][1], 1))
        if coords[2][0] > 0:
            x = coords[2][0] - 1
            vel = np.random.multivariate_normal([0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)
        elif coords[2][0] < 0:
            x = coords[2][0] + 1
            vel = np.random.multivariate_normal([0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2) * -1
        else:
            x = coords[2][0] 
            vel = np.random.multivariate_normal([0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)
        # position = np.array((x, y, 0))
        # vel[1] = 0

    print "x, y ", x, y
    position = np.array((x, y, 0))
    x, y, __ = (rotation_z_axis(position, blattoidea_euler[2]).T + blattoidea_position).reshape((3))

    model_vel = Twist()
    model_vel.linear.x = vel[0]
    model_vel.linear.y = vel[1]
    model_vel.linear.z = 0

    model_pose = Pose()
    model_pose.position.x = x
    model_pose.position.y = y
    model_pose.position.z = model_state.pose[ii].position.z
    model_pose.orientation.x = blattoidea_orientation[0]
    model_pose.orientation.y = blattoidea_orientation[1]
    model_pose.orientation.z = blattoidea_orientation[2]
    model_pose.orientation.w = blattoidea_orientation[3]
    
    model = ModelState()
    print "obj", obj
    model.model_name = obj
    model.twist = model_vel
    model.pose = model_pose
    # model.reference_frame = reference
    
    gazebo_model_srv(model)
    
def point_poly(model_state, obj):
    global coords
    polygon = geometry.Polygon(coords)
    inside_list = []
    for ii in xrange(len(obj)):
        idx = model_state.name.index(obj[ii])
        point = geometry.Point(model_state.pose[idx].position.x, model_state.pose[idx].position.y)
        within = point.within(polygon)
        inside_list.append(within)
    return inside_list

def camera_poly(poly_coords, frame):
    poly = PolygonStamped()
    poly.polygon.points = []

    poly_point = Point()
    poly_point.x = poly_coords[0][0]
    poly_point.y = poly_coords[0][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = poly_coords[1][0]
    poly_point.y = poly_coords[1][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = poly_coords[2][0]
    poly_point.y = poly_coords[2][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = poly_coords[3][0]
    poly_point.y = poly_coords[3][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)
    
    poly_point = Point()
    poly_point.x = poly_coords[4][0]
    poly_point.y = poly_coords[4][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly_point = Point()
    poly_point.x = poly_coords[5][0]
    poly_point.y = poly_coords[5][1]
    poly_point.z = 0.0
    poly.polygon.points.append(poly_point)

    poly.header.frame_id = frame
    poly.header.stamp = rospy.Time.now()
    return poly

def main():
    global coords
    rospy.init_node("advertise_polygon", anonymous=True)
    rospy.wait_for_service("gazebo/set_model_state")
    poly_pub = rospy.Publisher('/poly_pub', PolygonStamped, queue_size=2)
    gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
    # rospy.Subscriber('/poly_pub', PolygonStamped, poly_callback)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    x1 = 1.6
    y1 = np.sqrt(3) * x1 - x1 
    x2 = 5 
    y2 = np.sqrt(3) * x2 - x1
    poly_coords = [(x1, 0.0), (x1, y1), (x2, y2), (x2, -y2), (x1, -y1), (x1, 0.0)]
    coords = poly_coords
    poly = camera_poly(poly_coords, "base_linke")
    poly_pub.publish(poly)
    models_list = ["sphere_blue","sphere_green","sphere_red","sphere_yellow"]

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        obj = []
        for ii in range (len(model_state.name)):
            if model_state.name[ii] in models_list:
                obj.append(model_state.name[ii])
        blattoidea_idx = model_state.name.index ("blattoidea")
        poly = camera_poly(poly_coords, "base_linke")
        poly_pub.publish(poly)
        inside = point_poly(model_state, obj)
        obj_place(model_state, blattoidea_idx, obj = obj, inside = inside, gazebo_model_srv = gazebo_model_srv)
        # print "inside", inside

        r.sleep()

if __name__ == "__main__":
    main()
    