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
import threading
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

kp_distance = 1.1
ki_distance = 0.1
kd_distance = 0.5

kp_angle = 0.5
ki_angle = 0.0
kd_angle = 0.0

class path_Trajectory():
    def __init__(self):
        rospy.init_node('path_Trajectory', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        ## Set the odom frame self.odom_frame = '/odom'
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        # move robt after two second
        rospy.sleep(2)
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        # Initialize the position variable as a Point type
        self.dt = 0
        self.reached=False
        self.i=0

    # normalize angle
    def normalize_angle(self,angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def check_PID(self):
        # get path points
        xt, yt = self.getkey()

        for i in range(len(xt)):
            targetgoal = (float(xt[i]), float(yt[i]))
            print('target goal',targetgoal)
            self.pid_follower(targetgoal)

        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))
        theta_error= atan2((y_goal- position.y ),(x_goal -position.x ))
        theta_error= self.normalize_angle(theta_error)
        print("reached :)   ^_^")
        self.reached=True

        while abs(theta_error) > 0.05 :
            self.move_cmd.linear.x = 0.00
            self.move_cmd.angular.z = 0.0
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())



    def pid_follower(self, targetgoal):

        (x_goal , y_goal) = (targetgoal[0],targetgoal[1])
        # Get the starting position values
        (position, rotation) = self.get_odom()
        x_curr = position.x
        y_curr = position.y
        # Keep track of the distance traveled
        dist_error = sqrt(pow((x_goal -x_curr), 2) + pow(( y_goal -y_curr), 2))
        theta_error= atan2((y_goal- y_curr),(x_goal -x_curr))
        #theta_error_rotation= atan2((x_goal -position.x ),(y_goal- position.y ))


        # Enter the loop to move along a side
        while (dist_error ) > 0.005   and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            # Get the current position
            (position, rotation) = self.get_odom()
            # Compute the Euclidean distance from the start
            dist_error = sqrt(pow((x_goal -position.x ), 2) + pow(( y_goal -position.y ), 2))

            control_signal_distance = kp_distance*dist_error


            if control_signal_distance > 0.2 :
                control_signal_distance = 0.2
            self.move_cmd.linear.x = control_signal_distance
            #theta_error= atan2((y_goal- position.y ),(x_goal -position.x )) - self.normalize_angle(rotation)
            theta_error= atan2((y_goal- position.y ),(x_goal -position.x ))

            if  theta_error< 0:
                path = rotation -atan2((y_goal- position.y ),(x_goal -position.x ))
                path=self.normalize_angle(path)
                control_signal_angle = 2/pi* (path)
            else :
                path = atan2((y_goal- position.y ),(x_goal -position.x )) -rotation
                path=self.normalize_angle(path)
                control_signal_angle = 2/pi* (path)
            print('theta error',theta_error , 'rotation', rotation, 'path', path)



            if control_signal_angle > 0:
                control_signal_angle = min(control_signal_angle, 0.5)
                self.move_cmd.angular.z = control_signal_angle
            else:
                control_signal_angle = max(control_signal_angle, -0.5)
                self.move_cmd.angular.z = control_signal_angle








            #print('theta error',theta_error , 'rotation', rotation )
            # control linear velocity





            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

    def getkey(self):

        xt,yt =[] , []
        i=5
        X1 = np.linspace(0, 2,i)
        Y1 = np.array([0]*i)
        Y2 = np.linspace(0, 3 ,i)
        X2 = np.array([2]*i)
        X3 = np.linspace(2, -2 , i)
        Y3 = np.array([3]*i)
        Y4 = np.linspace(3, -3 , i)
        X4 = np.array([-2]*i)
        xt=np.concatenate([X1,X2])
        yt =np.concatenate([Y1,Y2])
        return xt, yt

    def get_odom(self):

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


def main():

    path=path_Trajectory()
    path.check_PID()
    r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
