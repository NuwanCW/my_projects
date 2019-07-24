#!/usr/bin/env python
'''
run these files first
1. roslaunch mybot_gazebo mybot_world.launch
2.roslaunch mybot_navigation gmapping_demo.launch
3.roslaunch mybot_description mybot_rviz_gmapping.launch
** follow this to install gmapping
https://answers.ros.org/question/300480/building-open_gmapping-from-source-on-melodicubuntu-1804/
'''
import message_filters
from message_filters import SimpleFilter
import Queue
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
hz = 30

# get callback function for distance i several angles


class TfMessageFilter(SimpleFilter):
    """Stores a message unless corresponding transforms is 
    available
    """

    def __init__(self, input_filter, base_frame, target_frame,
                 queue_size=1000):
        SimpleFilter.__init__(self)
        self.connectInput(input_filter)
        self.base_frame = base_frame
        self.target_frame = target_frame
        # TODO: Use a better data structure
        self.message_queue = Queue.Queue(maxsize=queue_size)
        self.listener = tf.TransformListener()
        self.max_queue_size = queue_size
        self._max_queue_size_so_far = 0

    def connectInput(self, input_filter):
        self.incoming_connection = \
            input_filter.registerCallback(self.input_callback)

    def poll_transforms(self, latest_msg_tstamp):
        """
        Poll transforms corresponding to all messages. If found throw older
        messages than the timestamp of transform just found
        and if not found keep all the messages.
        """
        global current_x, current_y, current_yaw, current_xy
        # Check all the messages for transform availability
        tmp_queue = Queue.Queue(self.max_queue_size)
        first_iter = True
        # Loop from old to new
        while not self.message_queue.empty():
            msg = self.message_queue.get()
            tstamp = msg.header.stamp
            if (first_iter and
                    self.message_queue.qsize() > self._max_queue_size_so_far):
                first_iter = False
                self._max_queue_size_so_far = self.message_queue.qsize()
                rospy.logdebug("Queue(%d) time range: %f - %f" %
                               (self.message_queue.qsize(),
                                tstamp.secs, latest_msg_tstamp.secs))
                # rospy.loginfo("Maximum queue size used:%d" %
                #               self._max_queue_size_so_far)
            if self.listener.canTransform(self.base_frame, self.target_frame,
                                          tstamp):
                (trans, quat) = self.listener.lookupTransform(self.base_frame,
                                                              self.target_frame, tstamp)
                current_x, current_y = round(trans[0] * 20.) / 20., round(
                    trans[1] * 20.) / 20.
                _, _, current_yaw = euler_from_quaternion(quat)
                current_xy = (current_x, current_y)
                current_point = (current_x, current_y, current_yaw)
                print(current_point)
                self.signalMessage(msg, (trans, quat))
                # Note that we are deliberately throwing away the messages
                # older than transform we just received
                return
            else:
                # if we don't find any transform we will have to recycle all
                # the messages
                tmp_queue.put(msg)
        self.message_queue = tmp_queue
        # print("from q",tmp_queue.queue[0])

    def input_callback(self, msg):
        """ Handles incoming message """
        if self.message_queue.full():
            # throw away the oldest message
            rospy.logwarn("Queue too small. If you this message too often"
                          + " consider increasing queue_size")
            self.message_queue.get()

        self.message_queue.put(msg)
        # This can be part of another timer thread
        # TODO: call this only when a new/changed transform
        self.poll_transforms(msg.header.stamp)


def callback(LaserScan, trans_rot):
    pass
    # print("Got LaserScan:%f" % LaserScan.header.stamp.secs)


def callbacklaser(msg):
    global front, back, left, right, right_ang, left_ang, scan_range, wall_right, wall_left
    front = round(min(msg.ranges[175:185]), 2)
    back = round(min(min(msg.ranges[-5:]), min(msg.ranges[:5])), 2)
    right = round(min(msg.ranges[85:95]), 2)
    left = round(min(msg.ranges[265:275]), 2)
    scan_range = min(right, left)


def poseCallback(pose_message):
    loop_rate = rospy.Rate(30)
    global roll, pitch, yaw, x, y
    orientation_q = pose_message.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    x = round(pose_message.pose.pose.position.x, 2)
    y = round(pose_message.pose.pose.position.y, 2)
    th = [pose_message.pose.pose.orientation.x, pose_message.pose.pose.orientation.y,
          pose_message.pose.pose.orientation.z, pose_message.pose.pose.orientation.w]
    # pose_message.pose.pose.orientation.w
    yaw = round(degrees(euler_from_quaternion(th)[-1]), 2)


def transformations(ret=False):
    global current_x, current_y, current_yaw, current_xy
    current_xy = (current_x, current_y)
    current_point = (current_x, current_y, current_yaw)
    # print "trna",trans[0],trans[1]
    return current_point


# move robot forward
def go_xy(x, y):
    loop_rate = rospy.Rate(hz)
    velocity_message = Twist()
    gain = 0.005
    # rotate(degrees(atan2(y - current_y, x - current_x) - current_yaw), -1)
    while True:
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

        if front < .5:
            # rotate(60, -1)
            obstacle_start_point = (current_x, current_y)
            print obstacle_start_point
            while True:
                ret = follow_wall(transformations, x,
                                  y, obstacle_start_point)
                if ret == -1:
                    print "I got ret"
                    break

        if abs(current_x-x) <= .05 and abs(current_y-y) <= .05:
            stop()
            break
        loop_rate.sleep()


def stop():
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    time.sleep(2)


def rotate(angle, dir):
    loop_rate = rospy.Rate(hz)
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
    go_xy(4, 0)
    stop()
    # rotate(0, -1)
    print "done"
    go_xy(4, 4)
    stop()
    # rotate(60, -1)
    print "done"
    go_xy(0, 4)
    stop()
    # rotate(60, -1)
    print "done"
    go_xy(0, 0)
    stop()
    # go_forward(90, 3)
    print "1st done"


if __name__ == '__main__':
    try:
        rospy.init_node('laser_data', anonymous=True)
        sub = rospy.Subscriber("/mybot/laser/scan", LaserScan, callbacklaser)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber("/odom", Odometry, poseCallback)
        time.sleep(2)
        # rospy.init_node('test_tf_sync_msgs', log_level=rospy.DEBUG)
        image_sub = message_filters.Subscriber('/mybot/laser/scan',
                                               LaserScan)

        ts = TfMessageFilter(image_sub, '/map', '/chassis',
                             queue_size=1500)
        ts.registerCallback(callback)
        # rospy.spin()

        execute_main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
