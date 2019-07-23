#!/usr/bin/env python
'''
run these files first
1. roslaunch mybot_gazebo mybot_world.launch
2.roslaunch mybot_navigation gmapping_demo.launch
3.roslaunch mybot_description mybot_rviz_gmapping.launch
** follow this to install gmapping
https://answers.ros.org/question/300480/building-open_gmapping-from-source-on-melodicubuntu-1804/
'''
from sensor_msgs.msg import LaserScan
# from scipy import spatial
import numpy as np
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import radians, degrees, atan2, sqrt, pi, sin, cos
from pid import PID
import rospy
from wall_follow import follow_wall  # , straight_follow
import roslib
import tf
import traceback
import json
import time
import random
import math


controller = PID(2.30, 0.8, 1.3, .8, -.8)
velocity_message = Twist()

global front, left, right, current_x, current_y, current_yaw, current_xy, boundary1, boundar2, boundary3, boundary4

front, left, right, current_x, current_y, current_yaw, current_xy = 0, 0, 0, 0, 0, 0, (
    0, 0)
boundary1, boundar2, boundary3, boundary4 = [], [], [], []


# get callback function for distance i several angles
def callbacklaser(msg):
    global front, back, left, right, right_ang, left_ang, scan_range, wall_right, wall_left
    front = round(min(msg.ranges[175:185]), 2)
    back = round(min(min(msg.ranges[-5:]), min(msg.ranges[:5])), 2)
    right = round(min(msg.ranges[85:95]), 2)
    left = round(min(msg.ranges[265:275]), 2)
    scan_range = min(right, left)


def transformations(ret=False):
    global current_x, current_y, current_yaw, current_xy
    now = rospy.Time(0)
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/chassis", now, rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform("/map", "/chassis", now)
    current_x, current_y = round(trans[0] * 20.) / 20., round(
        trans[1] * 20.) / 20.
    _, _, current_yaw = euler_from_quaternion(rot)
    current_xy = (current_x, current_y)
    current_point = (current_x, current_y, current_yaw)
    if ret == True:
        return current_point


# move robot forward
def go_xy(x, y):
    transformations()
    loop_rate = rospy.Rate(30)
    velocity_message = Twist()
    gain = 0.005
    while True:
        transformations()
        error_angle = atan2(y - current_y, x - current_x) - current_yaw
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
        loop_rate.sleep()
        # print(current_x, current_y, front)
        if front < .5:
            obstacle_start_point = (current_x, current_y)
            print obstacle_start_point
            while True:
                ret = follow_wall(transformations, True, x,
                                  y, obstacle_start_point)
                if ret == -1:
                    print "I got ret"
                    break
                # velocity_publisher.publish(m)
                # loop_rate.sleep()
                # if (current_x == x or current_y == y):
                #     stop()
                #     print(current_x, x, current_y, y)
                #     break
                # else:
                #     print "follwing wall"

        if abs(current_x-x) <= .05 and abs(current_y-y) <= .05:
            stop()
            print "done"
            break


def stop():
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def rotate(angle, dir):
    loop_rate = rospy.Rate(30)
    while True:
        now = rospy.Time(0)
        transformations()
        error_angle = radians(angle) - current_yaw
        error_angle = error_angle + 2 * pi if (
            error_angle <= -pi) else error_angle - 2 * pi if (
                error_angle >= pi) else error_angle
        velocity_message.linear.x = 0.025
        # if abs(degrees(error_angle))>2 else max(np.sign(error_angle)*.2,controller.update_PID(error_angle)) if error_angle<0 else min(0.2,controller.update_PID(error_angle))
        velocity_message.angular.z = 0.35 * dir
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        if abs(degrees(error_angle)) < 2:
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            velocity_publisher.publish(velocity_message)
            c_cordinates = (round(current_x * 4.) / 4.,
                            round(current_y * 4.) / 4.)
            # current_row.append(c_cordinates)
            # wall_list=fill
            # fill=list(set(fill + current_row))
            # del current_row[:]
            print("rotation done")
            time.sleep(0.20)  # 0.5
            return


def execute_main():
    # follow_wall()
    go_xy(0, 0)
    # go_xy(4, 4)
    # go_xy(0, 4)
    # go_xy(0, 0)
    # go_forward(90, 3)
    print "1st done"


if __name__ == '__main__':
    try:
        rospy.init_node('laser_data', anonymous=True)
        sub = rospy.Subscriber("/mybot/laser/scan", LaserScan, callbacklaser)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        time.sleep(2)
        execute_main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
