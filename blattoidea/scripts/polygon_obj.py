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

def spawn_square(gazebo_model_srv, obj, reference, mean_vel = [0, 0], cov_vel = [[6, 0], [0, 6]]):
    global coords, model_state
    blattoidea_idx = model_state.name.index ("blattoidea")
    blattoidea_orientation = np.array((model_state.pose[blattoidea_idx].orientation.x,model_state.pose[blattoidea_idx].orientation.y, model_state.pose[blattoidea_idx].orientation.z, model_state.pose[blattoidea_idx].orientation.w))

    rand = int(np.random.randint(4, size=1))
    if rand is 0:
        x = float(np.random.uniform(coords[1][0], coords[2][0], 1))
        y = coords[1][1] - 1
        if x >= 0:
            vel = np.random.multivariate_normal([-0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)
        elif x < 0:
            vel = np.random.multivariate_normal([0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)    
    elif rand is 1:
        x = float(np.random.uniform(coords[1][0], coords[2][0], 1))
        y = coords[3][1] + 1
        if x >= 0:
            vel = np.random.multivariate_normal([-0.5, 4], [[1, 0], [0, 1]], (1)).reshape(2)
        elif x < 0:
            vel = np.random.multivariate_normal([0.5, 4], [[1, 0], [0, 1]], (1)).reshape(2)     
    elif rand is 2:
        y = float(np.random.uniform(coords[3][1], coords[1][1], 1))
        x = coords[2][0] - 1
        if y >= 0:
            vel = np.random.multivariate_normal([-4, -0.5], [[1, 0], [0, 1]], (1)).reshape(2)
        elif y < 0:
            vel = np.random.multivariate_normal([-4, 0.5], [[1, 0], [0, 1]], (1)).reshape(2)
    else:
        y = float(np.random.uniform(coords[3][1], coords[1][1], 1))
        x = coords[1][0] + 1
        if y >= 0:
            vel = np.random.multivariate_normal([4, -0.5], [[1, 0], [0, 1]], (1)).reshape(2)
        elif y < 0:
            vel = np.random.multivariate_normal([4, 0.5], [[1, 0], [0, 1]], (1)).reshape(2)

    print "x, y, rand ", x, y, rand

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
    print "obj", obj
    model.model_name = obj
    model.twist = model_vel
    model.pose = model_pose
    
    gazebo_model_srv(model)

def spawn_poly(gazebo_model_srv, obj, reference, mean_vel = [0, 0], cov_vel = [[6, 0], [0, 6]]):
    global coords, model_state
    blattoidea_idx = model_state.name.index ("blattoidea")
    # blattoidea_position = np.array((model_state.pose[blattoidea_idx].position.x, model_state.pose[blattoidea_idx].position.y, model_state.pose[blattoidea_idx].position.z))
    blattoidea_orientation = np.array((model_state.pose[blattoidea_idx].orientation.x,model_state.pose[blattoidea_idx].orientation.y, model_state.pose[blattoidea_idx].orientation.z, model_state.pose[blattoidea_idx].orientation.w))
    # blattoidea_euler = np.array(euler_from_quaternion(blattoidea_orientation))
 
    rand = int(np.random.randint(3, size=1))
    if rand is 0:
        x = float(np.random.uniform(coords[1][0], coords[2][0], 1))
        y = float((np.sqrt(3) * x - 1))
        vel = np.random.multivariate_normal([-0.5, -4], [[1, 0], [0, 1]], (1)).reshape(2)
    elif rand is 1:
        x = float(np.random.uniform(coords[1][0], coords[2][0], 1))
        y = float((np.sqrt(3) * x - 1) * (-1))
        vel = np.random.multivariate_normal([-0.5, 4], [[1, 0], [0, 1]], (1)).reshape(2)
    elif rand is 2:
        y = float(np.random.uniform(coords[3][1], coords[2][1], 1))
        x = coords[2][0] - 1
        if y >= 0:
            vel = np.random.multivariate_normal([-4, -0.5], [[1, 0], [0, 1]], (1)).reshape(2)
        elif y < 0:
            vel = np.random.multivariate_normal([4, 0.5], [[1, 0], [0, 1]], (1)).reshape(2) * -1

    print "x, y, rand ", x, y, rand
    # position = np.array((x, y, 0))
    # x, y, __ = (rotation_z_axis(position, blattoidea_euler[2]).T + blattoidea_position).reshape((3))
    # vel = np.array((vel[0], vel[1], 0))
    # velx, vely, __ = (rotation_z_axis(vel, blattoidea_euler[2]).T).reshape((3))
    # vel = np.array((velx, vely))

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
    print "obj", obj
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
            except tf2_ros.TransformException as (e):
                print "some tf2 exception happened", e
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

def main():
    global coords, model_state
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    poly_pub = rospy.Publisher('/poly_pub', PolygonStamped, queue_size=2)
    gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
    
    x1 = 1.0
    y1 = np.sqrt(3) * x1 + 1 
    x2 = 6 
    y2 = np.sqrt(3) * x2 
    poly_coords = [(x1, 0.0), (x1, y1), (x2, y2), (x2, -y2), (x1, -y1), (x1, 0.0)]
    x1 = -10
    y1 = 10  
    x2 = 10 
    y2 = -10 
    square_coords = [(x1, 0.0), (x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, 0.0)]
    
    coords = poly_coords
    poly = camera_poly("base_footprint")
    poly_pub.publish(poly)
    models_list = ["sphere_blue","sphere_green","sphere_red","sphere_yellow"]
    model_list_2 = ["cylinder_blue", "cylinder_green", "cylinder_red", "cylinder_yellow"]

    r = rospy.Rate(2)
    model_state = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    tf.TransformListener().waitForTransform("world", "base_footprint", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        poly = camera_poly("base_footprint")
        poly_pub.publish(poly)

        obj_poly = []
        obj_square = []
        for ii in range (len(model_state.name)):
            if model_state.name[ii] in models_list:
                obj_poly.append(model_state.name[ii])
            elif model_state.name[ii] in model_list_2:
                obj_square.append(model_state.name[ii])
        inside_poly = point_poly(tfBuffer, obj_poly, "base_footprint", poly_coords, True)
        inside_square = point_poly(tfBuffer, obj_square, "base_footprint", square_coords, False)

        for ii in range (len(obj_poly)):
            if not inside_poly[ii]:
                try:
                    spawn_poly(gazebo_model_srv, obj = obj_poly[ii], reference = "base_footprint")
                except:
                    rospy.logwarn("Spawn model " + obj_poly[ii] + " Failed!!")
                    pass
        for ii in range (len(obj_square)):
            if not inside_square[ii]:
                try:
                    spawn_square(gazebo_model_srv, obj = obj_square[ii], reference = "base_footprint")
                except:
                    rospy.logwarn("Spawn model " + obj_square[ii] + " Failed!!")
                    pass
        # print "inside", inside
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("advertise_polygon", anonymous=True)
    rospy.wait_for_service("gazebo/set_model_state")
    # rospy.Subscriber('/poly_pub', PolygonStamped, poly_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
    
    main()
    