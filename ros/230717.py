#!/usr/bin/env python
'''
run these files first
1. roslaunch mybot_gazebo mybot_world.launch
2.roslaunch mybot_description mybot_rviz_gmapping.launch
3.roslaunch mybot_description mybot_rviz_gmapping.launch
** follow this to install gmapping
https://answers.ros.org/question/300480/building-open_gmapping-from-source-on-melodicubuntu-1804/
'''

import sys
sys.path.insert(0, '/home/nuwan/projects/ros/custom_modules')
from sensor_msgs.msg import LaserScan
from scipy import spatial
import numpy as np
import rospy, roslib, tf, sys, traceback, json, time, random, math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from math import radians, degrees, atan2, sqrt, pi, sin, cos
from pid import PID
from wall_follow import mains, straight_follow
controller = PID(2.30, 0.8, 1.3, .8, -.8)
velocity_message = Twist()
# global front,back,left,right,roll,boundry1,remember_list,boundry2,boundry3,wall_left,i,wall_right, pitch, yaw,x,y,yaww,last_distance,right_ang,left_ang,scan_range,follow_wall,first_column,fill,current_row,ylength,wall_list, r,wall_dist
global front, left, right, xx, yy, yaww  #roll,boundry1,remember_list,boundry2,boundry3,wall_left,i,wall_right, pitch, yaw,x,y,yaww,last_distance,right_ang,left_ang,scan_range,follow_wall,first_column,fill,current_row,ylength,wall_list, r,wall_dist
follow_wall = False
last_distance, scan_range, ylength, first_column, i = 0, 0, 0, 0, 0

fill,obstacles,missing,remember_list,current_row,wall_list,boundry1,boundry2,boundry3=[],[],[],[],[],[],[],[],[]


# get callback function for distance i several angles
def callbacklaser(msg):
    # initialise the distance in different sides
    global front, back, left, right, scan_range
    front = round(min(msg.ranges[175:185]), 2)
    back = round(min(min(msg.ranges[-5:]), min(msg.ranges[:5])), 2)
    right = round(min(msg.ranges[85:95]), 2)
    left = round(min(msg.ranges[265:275]), 2)
    scan_range = min(right, left)
    # print("scan is:",front,right,back,left)


# get the real transformed coordinates and angle of the robot
def transformations(ret=False):
    global xx, yy, yaww
    now = rospy.Time(0)
    listener.waitForTransform("/map", "/chassis", now, rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform("/map", "/chassis", now)
    xx, yy = round(trans[0] * 20.) / 20., round(trans[1] * 20.) / 20.
    _, _, yaww = euler_from_quaternion(rot)
    # if ret==True:
    # return round(xx*20.)/20.,round(yy*20.)/20.,yaww


# move robot forward
def go_forward(desired_yaw_deg):
    global xx, yy, yaww
    # initialise the global variables
    # global fill,remember_list,current_row,wall_list,last_distance,i,boundry1,boundry2,boundry3,yy, yaww,x,y,xx,yaw,front,back,left,right,right_ang,left_ang,scan_range,follow_wall,first_column

    loop_rate = rospy.Rate(30)
    transformations()
    # get the initial update for the transformed coordinates
    cx, cy, yaww = xx, yy, yaww
    distance = 0.0
    # fill.append((xx,yy))
    now = rospy.Time(0)
    rospy.sleep(0.20)
    gain = 0.005
    # boundry=boundry1 if forward==True else boundry3
    # filled_crd = spatial.KDTree(boundry)
    while True:
        now = rospy.Time(0)
        transformations()
        error_angle = radians(desired_yaw_deg) - yaww
        error_angle = error_angle + 2 * pi if (
            error_angle <= -pi) else error_angle - 2 * pi if (
                error_angle >= pi) else error_angle
        if gain < 0.4 and front > .2:
            gain += 0.005
        else:
            gain -= .008
        gain = max(gain, .21)
        velocity_message.linear.x = max(
            (12 - abs(degrees(error_angle))) / 15.0, 0) * min(gain, .3627)
        velocity_message.angular.z = controller.update_PID(
            error_angle) * .9 if abs(
                degrees(error_angle)) <= 12 else np.sign(error_angle) * 0.4
        velocity_publisher.publish(velocity_message)
        # filled_cordinates1=(xx,yy-.1*np.sign(check))
        # c_cordinates=filled_cordinates1[0],filled_cordinates1[1]+.1*np.sign(check)
        distance = sqrt((cx - xx)**2 + (cy - yy)**2)
        loop_rate.sleep()
        if distance >= 2.0:
            stop()
            print "done"
            break


def stop():
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_xy(x_dis, y_dis, heading):
    loop_rate = rospy.Rate(30)
    global yy, yaww, xx, scan_range, wall_right, wall_left
    boundry = []
    now = rospy.Time(0)
    transformations()
    cx, cy = 0, 0
    x_goal, y_goal = cx + x_dis, cy + y_dis
    xc_dis, yc_dis = 0, 0
    rospy.sleep(0.20)
    while True:
        now = rospy.Time(0)
        transformations()
        if scan_range > .4:  # earlier this was scan range
            # print("scan_range",round(scan_range,1))
            desired_angle_goal = atan2(y_goal - yy, x_goal - xx)
            error_angle = desired_angle_goal - yaww
            error_angle = error_angle + 2 * pi if (
                error_angle <= -pi) else error_angle - 2 * pi if (
                    error_angle >= pi) else error_angle
            velocity_message.linear.x = max(
                (10 - abs(degrees(error_angle))) / 15.0, 0) * .3
            velocity_message.angular.z = controller.update_PID(
                error_angle) if abs(
                    degrees(error_angle)) < 10 else np.sign(error_angle) * 0.35
            velocity_publisher.publish(velocity_message)
            xc_dis, yc_dis = xx - cx, yy - cy
            print(xx, yy)
            boundry.append((xx, yy))
            loop_rate.sleep()
            if abs(xc_dis - x_dis) < .05 and abs(yc_dis - y_dis) < .05:
                stop()
                boundry = list(set(boundry))
                boundry = sorted(boundry, key=lambda k: [k[0], k[1]])
                return boundry
        else:
            print("scan_range from wall", round(scan_range, 1))
            rotate(heading + 60, 1)
            while True:
                updated_boundry, token = straight_follow(
                    transformations, True, cx, cy, x_dis, y_dis, heading,
                    boundry)
                boundry = list(set(boundry + updated_boundry))
                if token == -1:
                    boundry = list(set(boundry))
                    boundry = sorted(boundry, key=lambda k: [k[0], k[1]])
                    return boundry
                else:
                    transformations()
                    break


if __name__ == '__main__':
    try:
        rospy.init_node('laser_data', anonymous=True)
        pose_subscriber = rospy.Subscriber("/mybot/laser/scan", LaserScan,
                                           callbacklaser)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # pose_subscriber_ = rospy.Subscriber("/odom", Odometry, poseCallback)
        time.sleep(2)
        now = rospy.Time(0)
        listener = tf.TransformListener()
        listener.waitForTransform("/map", "/chassis", now, rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/map", "/chassis", now)
        xx, yy = round(trans[0], 1), round(trans[1], 1)
        _, _, yaww = euler_from_quaternion(rot)
        print("initial_yaaw", degrees(yaww), xx, yy)
        go_forward(90)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")