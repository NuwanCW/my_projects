#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
from scipy import spatial
import numpy as np
import rospy,roslib,tf,sys, traceback,json,time,random,math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from math import radians,degrees,atan2,sqrt,pi,sin,cos
from pid import PID
from wall_follow import mains,straight_follow
controller =PID(2.30,0.8,1.3,.8,-.8)
velocity_message = Twist()
global front,back,left,right,roll,boundry1,remember_list,boundry2,boundry3,wall_left,i,wall_right, pitch, yaw,x,y,yaww,last_distance,right_ang,left_ang,scan_range,follow_wall,first_column,fill,current_row,ylength,wall_list, r,wall_dist
follow_wall=False
last_distance,scan_range,ylength,first_column,i=0,0,0,0,0

fill,obstacles,missing,remember_list,current_row,wall_list,boundry1,boundry2,boundry3=[],[],[],[],[],[],[],[],[]

# get callback function for distance i several angles
def callbacklaser(msg):
    # initialise the distance in different sides
    global front,back,left,right,right_ang,left_ang,scan_range,wall_right,wall_left
    # front,back,left,right=round(msg.ranges[180],2),round(msg.ranges[0],2),round(msg.ranges[270],2),round(msg.ranges[90],2)
    back,left,right=round(msg.ranges[0],2),round(msg.ranges[270],2),round(msg.ranges[90],2)
    # right_ang,left_ang=round(msg.ranges[135],2),round(msg.ranges[225],2)
    right_ang,left_ang=min(msg.ranges[160:165]),min(msg.ranges[195:200])
    wall_left,wall_right=min(msg.ranges[270:350]),min(msg.ranges[90:170])
    # scan_range=min(msg.ranges[152:208])
    # front=min(msg.ranges[170:190])
    front=min(msg.ranges[160:200])
    scan_range=front#min(right_ang,left_ang)


# get the real transformed coordinates and angle of the robot
def transformations(ret=False):
    global xx,yy,yaww
    now=rospy.Time(0)
    listener.waitForTransform("/map", "/base_link",now,rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform("/map", "/base_link", now)
    xx,yy=round(trans[0]*20.)/20.,round(trans[1]*20.)/20.
    _,_,yaww=euler_from_quaternion(rot)
    # print(xx,yy)
    if ret==True:
        return round(xx*20.)/20.,round(yy*20.)/20.,yaww  

# get the coordinates when receive by the callback function
def poseCallback(pose_message):
    loop_rate = rospy.Rate(30)
    global roll, pitch, yaw,x,y
    orientation_q = pose_message.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x= round(pose_message.pose.pose.position.x,2)
    y= round(pose_message.pose.pose.position.y,2)
    th=[pose_message.pose.pose.orientation.x,pose_message.pose.pose.orientation.y,pose_message.pose.pose.orientation.z,pose_message.pose.pose.orientation.w]
    yaw=round(degrees(euler_from_quaternion(th)[-1]),2)#pose_message.pose.pose.orientation.w
    


    # move robot forward
def go_forward(desired_yaw_deg,check,forward):
    # initialise the global variables
    global fill,remember_list,current_row,wall_list,last_distance,i,boundry1,boundry2,boundry3,yy, yaww,x,y,xx,yaw,front,back,left,right,right_ang,left_ang,scan_range,follow_wall,first_column
    
    loop_rate = rospy.Rate(30)
    transformations()
    # get the initial update for the transformed coordinates
    cx,cy,yaww=xx,yy,yaww
    distance=0.0
    fill.append((xx,yy))
    now = rospy.Time(0)
    rospy.sleep(0.20)
    distance =0
    gain=0.005
    boundry=boundry1 if forward==True else boundry3
    filled_crd = spatial.KDTree(boundry) 
    while True:
        now=rospy.Time(0)
        transformations()
        error_angle = radians(desired_yaw_deg)-yaww
        error_angle = error_angle+2*pi if (error_angle <=-pi) else error_angle-2*pi if(error_angle >=pi) else error_angle
        if gain<0.4 and front>.2: 
            gain +=0.005
        else:
            gain -=.008
        gain = max(gain,.21)
        velocity_message.linear.x = max((12-abs(degrees(error_angle)))/15.0,0)*min(gain,.3627)
        velocity_message.angular.z = controller.update_PID(error_angle)*.9 if abs(degrees(error_angle))<=12 else  np.sign(error_angle)*0.4
        velocity_publisher.publish(velocity_message)
        filled_cordinates1=(xx,yy-.1*np.sign(check))
        c_cordinates=filled_cordinates1[0],filled_cordinates1[1]+.1*np.sign(check)
        distance= sqrt((cx-xx)**2+(cy-yy)**2)
        # print("Filled: {}, filled1 :{}".format(c_cordinates,filled_cordinates1))
        # if c_cordinates not in fill:
        current_row.append(c_cordinates)
        # rot_ang, tot is needed to rotate robot when it find missing column
        if forward==True and np.sign(check)>0:
            val=right 
            rot=-1
            rot_ang=degrees(yaww)-45
        if forward==False and np.sign(check)>0:
            val=left
            rot=1
            rot_ang=degrees(yaww)+45

        if forward==True and np.sign(check)<0:
            val=left
            rot=1
            rot_ang=degrees(yaww)+45
        if forward==False and np.sign(check)<0:
            val=right
            rot=-1
            rot_ang=degrees(yaww)-45
        
        # if filled_cordinates1 not in fill and val<.5 and first_column==1:  # this needs to be checked, this triggered at the first u=instancs since the data not updated in the list
        if filled_cordinates1 not in wall_list and val<.4 and first_column==1:  # this needs to be checked, this triggered at the first u=instancs since the data not updated in the list
            print("val:",val) 
            rotate(rot_ang,rot)
            # k=1 if val==left else -1
            k=1 if Forward==False else -1
            print("remember list to pass",remember_list)
            angl=mains(transformations,True,remember_list,k,left_d=True) # earlier the list was fill, now thw the wall list
            del remember_list[:]
            print("follow wall done")
            # follow_wall=False
            rotate(angle-degrees(angl),1)
            # break
        # print("front {} right_ang{} ,left_ang{}, scan_range{}".format(front,scan_range,right_ang,left_ang))
        loop_rate.sleep()
        # boundry=boundry1 if forward==True else boundry3
        # filled_crd = spatial.KDTree(boundry)
        cl_distance=round(filled_crd.query([c_cordinates])[0][0],2)
        print("boundry distance",cl_distance)
        if cl_distance<=.23 or front<0.45:# or scan_range<=.4:
            if cl_distance>.23 and len(remember_list)==0:
                rml=list(set(wall_list[:]))  # remember list is the y column lastly finished to full within boundraies, this is the target to come when following the wall
                remember_list=sorted(rml, key=lambda k: [k[0], k[1]])


                print("remember list updated and",remember_list)
            print(sorted(list(set(current_row)),key=lambda k:[k[0],k[1]]))
            fill=list(set(fill + current_row))

            wal=list(set(current_row))
            wall_list=sorted(wal, key=lambda k: [k[0], k[1]])
            # remember_list=current_row
            del current_row[:]
            # print("front {} scan_range{}".format(front,scan_range))
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            velocity_publisher.publish(velocity_message)
            first_column=1
            print("done")
            break
def stop():
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_xy(x_dis,y_dis,heading):
    loop_rate = rospy.Rate(30)
    global yy, yaww,xx,scan_range,wall_right,wall_left
    boundry=[]
    now=rospy.Time(0)
    transformations()
    cx,cy=0,0
    x_goal,y_goal=cx+x_dis,cy+y_dis
    xc_dis,yc_dis=0,0
    rospy.sleep(0.20)
    while True:
        now=rospy.Time(0)
        transformations()
        if scan_range>.4: # earlier this was scan range
            # print("scan_range",round(scan_range,1))
            desired_angle_goal = atan2(y_goal-yy, x_goal-xx)
            error_angle = desired_angle_goal-yaww
            error_angle = error_angle+2*pi if (error_angle <=-pi) else error_angle-2*pi if(error_angle >=pi) else error_angle
            velocity_message.linear.x = max((10-abs(degrees(error_angle)))/15.0,0)*.3
            velocity_message.angular.z = controller.update_PID(error_angle) if abs(degrees(error_angle))<10 else  np.sign(error_angle)*0.35
            velocity_publisher.publish(velocity_message)
            xc_dis,yc_dis=xx-cx,yy-cy
            print(xx,yy)
            boundry.append((xx,yy))
            # print("xc_dis,yc_dis ",round(xc_dis,2),round(yc_dis,2))
            loop_rate.sleep()
            if abs(xc_dis-x_dis)<.05 and abs(yc_dis-y_dis)<.05:
                stop() 
                print("done")
                # print(boundry)
                return boundry
        else:
            print("scan_range from wall",round(scan_range,1))
            rotate(heading+60,1)
            while True:
                updated_boundry,token=straight_follow(transformations,True,cx,cy,x_dis,y_dis,heading,boundry)
                boundry = list(set(boundry+updated_boundry))
                if token==-1:
                    # print(boundry)
                    return boundry
                else:
                    transformations()
                    # rotate(degrees(yaww)-heading,1)
                    break

                    
          


def draw_sq():
    global boundry1,boundry2,boundry3
    print("1st sq")
    b1=go_xy(1.50,0.0,0)
    b1=list(set(b1))
    boundry1=sorted(b1, key=lambda k: [k[0], k[1]])
    print(boundry1)
    time.sleep(2)
    print("2nd sq")
    # boundry2=list(set(sorted(go_xy(1.50,1.50,90))))
    b2=go_xy(1.50,1.50,90)
    b2=list(set(b2))
    boundry2=sorted(b2, key=lambda k: [k[0], k[1]])
    print(boundry2)
    time.sleep(2)
    print("3rd sq")
    # boundry3=list(set(sorted(go_xy(0.0,1.50,180))))
    b3=go_xy(0.0,1.50,180)
    b3=list(set(b3))
    boundry3=sorted(b3, key=lambda k: [k[0], k[1]])
    print(boundry3)
    time.sleep(2)
    # print("4th sq")
    # go_xy(0.0,0.0,270)
    rotate(270,1)
    

#####################################################################################################################
def go_to_distance(dis):
    loop_rate = rospy.Rate(30)
    global yy, yaww,x,y,xx,yaw
    now=rospy.Time(0)
    transformations()
    cx,cy=round(xx*4.)/4.,round(yy*4.)/4.
    ox,oy=xx,yy
    # _,_,yaw=euler_from_quatecrnion(rot)
    # x_goal,y_goal=xadd+cx,yadd+cy
    now = rospy.Time(0)
    rospy.sleep(0.20)
    distance =0
    while True:
        now=rospy.Time(0)
        transformations()
        distance=sqrt((cx-xx)**2+(cy-yy)**2)
        # ls =max((10-abs(degrees(error_angle)))/15.0,0)*.3
        linear_speed = .15#min(int(abs(degrees(error_angle))<4)*ls*1.1+int(abs(degrees(error_angle))>=4)*ls*.9,0.3)
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = 0.05#angular_speed
        velocity_publisher.publish(velocity_message)
        print("distance ",distance)
        loop_rate.sleep()
        if distance>dis: 
            print("done")
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            velocity_publisher.publish(velocity_message)
            print("done")
            # time.sleep(1)
            # loop_rate.sleep()
            break

def rotate(angle,dir):
    global fill,current_row
    loop_rate = rospy.Rate(30)
    fill=list(set(fill + current_row))
    while True:
        now=rospy.Time(0)
        transformations()
        error_angle = radians(angle)-yaww
        error_angle = error_angle+2*pi if (error_angle <=-pi) else error_angle-2*pi if(error_angle >=pi) else error_angle
        velocity_message.linear.x = 0.025
        velocity_message.angular.z = 0.35*dir# if abs(degrees(error_angle))>2 else max(np.sign(error_angle)*.2,controller.update_PID(error_angle)) if error_angle<0 else min(0.2,controller.update_PID(error_angle))
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        if abs(degrees(error_angle))<2: 
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            velocity_publisher.publish(velocity_message)
            c_cordinates=(round(xx*4.)/4.,round(yy*4.)/4.)
            current_row.append(c_cordinates)
            wall_list=fill
            fill=list(set(fill + current_row))
            del current_row[:] 
            print("rotation done")
            time.sleep(0.20)#0.5
            return

def bou(left_d=True):
    global yy, yaww,xx
    move_d=1 if left_d==True else -1
    first_move=left if left_d==True else right
    second_move=right if left_d==True else left
    global follow_wall,ylength,fill,wall_list
    forward=True
    go_forward(degrees(ca),move_d,forward)
    rotate(angle,move_d)
    forward=False
    go_forward(angle,move_d,forward)
    # if follow_wall==True:
    #     return -1
    # if second_move>.25:
    rotate(degrees(ca),move_d*(-1))
    # ylength +=.18
    if round(yy*4.)/4.>1.5:
        return -1
    return 0


if __name__ == '__main__':
    try:      
        rospy.init_node('laser_data', anonymous=True)
        pose_subscriber = rospy.Subscriber("/scan", LaserScan, callbacklaser)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        # pose_subscriber = rospy.Subscriber("/odom", Odometry, poseCallback) 
        time.sleep(2)
        now=rospy.Time(0)
        listener = tf.TransformListener()
        listener.waitForTransform("/map", "/base_link",now,rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/map", "/base_link", now)
        xx,yy=round(trans[0],1),round(trans[1],1)
        _,_,yaww=euler_from_quaternion(rot)
        print("initial_yaaw",degrees(yaww))
        print("draw_sq")
        draw_sq()
        fill.append((xx,yy))
        current_row.append((xx,yy))
        wall_list.append((xx,yy))
        print("fill and curent_row",fill,current_row)
        missing.append((xx,yy+.25))
        ca=yaww
        angle=degrees(ca)+180
        ylength=0
        while True:
            v=bou()
            if v==-1:
                break
        # wall_list=list(set(fill)-set(current_row)
        # fill=list(set(fill + current_row))
        
        del current_row[:] 
        # while True:
        print("length of fill and wall",len(fill),len(wall_list))
        # left_d=True his is important to see wheter we are covering left or right
        # mains(transformations,True,wall_list,left_d=True)
        # print("testing")
        ang=ca
        rotate(degrees(ang),1)
        # follow_wall=False
        first_column=0
        while True:
            print("second loop")
            v=bou()
            if v==-1:
                break
        print("seems done")

        # req_ang=degrees(atan2(obstacles[0][1],obstacles[0][0]))
        # print("obs corrdinates",obstacles[0],"req_ang",req_ang,"yaw",yaww)
        # rotate(req_ang,1)


        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
