#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import subprocess
import os
import time
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
import time
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

"""
kp_distance = 10
ki_distance = 0.01
kd_distance = 0.5

p_distance = 5
ki_distance = 0.01
kd_distance = 0.5
#...........
kp_angle = 1.2
ki_angle = 0.005
kd_angle = 0.04

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


class GotoPoint():
    def __init__(self):
        rospy.init_node('pidnavigation.py', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.reached=False
        self.x_error=[]
        self.y_error=[]
        self.t_p=[]

        #path
        self.path = Path()
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size=10)
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()


        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        #
        xt, yt = self.getkey()

        for i in range(len(xt)):
            (goal_x, goal_y) = [float(xt[i]), float(yt[i])]

            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            #distance is the error for length, x,y
            distance = goal_distance
            previous_distance = 0
            total_distance = 0
            previous_angle = 0
            total_angle = 0


            while distance > 0.02 :
                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y
                #path_angle = error
                path_angle = atan2(goal_y - y_start, goal_x- x_start)- rotation
                path_angle=abs(normalize_angle(path_angle))

                diff_angle = path_angle - previous_angle
                diff_distance = distance - previous_distance

                distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

                #PID FORMULA FOR LINEAR VELOCITY

                control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

                #PORPORTIONAL CONTROLLER FOR ANGLUAR VELOCITY

                control_signal_angle = kp_angle*(path_angle)

                move_cmd.angular.z = (control_signal_angle)
                #move_cmd.linear.x = min(linear_speed * distance, 0.1)
                move_cmd.linear.x = min(control_signal_distance, 0.4)

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.1)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.1)


                self.cmd_vel.publish(move_cmd)
                r.sleep()
                last_rotation = rotation
                previous_distance = distance
                total_distance +=  distance
                print("Current positin and rotation are: ", (position, rotation))

            self.x_error.append(goal_x - x_start)
            self.y_error.append(goal_y - y_start)
            self.t_p.append(time.clock_gettime(3))


        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("reached :)   ^_^")
        self.reached=True
        while abs(rotation - path_angle) > 0.05 :
            (position, rotation) = self.get_odom()
            move_cmd.linear.x = 0.00
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd)

        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())





    def getkey(self):

        xt=[2,2,-2,-2, 2,2]
        yt=[0,3,3,-3,-3,0]

        return xt, yt

    def get_odom(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    def plottest(self):
        t=[]
        t=self.t_p
        plt.plot(t,self.x_error)
        plt.xlabel('time')
        plt.ylabel('x error')
        plt.title('X Error')
        plt.grid(True)
        plt.show()
        plt.plot(t,self.y_error)
        plt.xlabel('time')
        plt.ylabel('y error')
        plt.title('Y Error')
        plt.grid(True)
        plt.show()
    def pathplot(self):
        if self.reached != False:
            self.plottest()
            show()

def main():

    m=GotoPoint()
    m.pathplot()
    show()
    r.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
