#!/usr/bin/env python3

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi
import PyKDL
from math import pi
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

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class NavSquare():
    def __init__(self):
        # Give the node a name
        rospy.init_node('nav_square', anonymous=False)
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        # How fast will we check the odometry values?
        rate = 20
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        initial_position=np.array([2, 0, 0])
        #print('(X, Y, Theta):' ,coord[0], coord[1], angle[0])
        print('Initial pose is:-')
        print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])
        q = quaternion_from_euler(0, 0, initial_position[2])
        # state_msg is an object
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_waffle'
        state_msg.pose.position.x = initial_position[0]
        state_msg.pose.position.y = initial_position[1]
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        print(resp)
        time.sleep(5)
        
        # Set the parameters for the target square
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians
        linear_speed = rospy.get_param("~linear_speed", input())        # meters per second
        angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2)) # degrees to radians

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        self.x_error=[]
        self.y_error=[]
        self.t_p=[]
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        # Set the odom frame
        self.odom_frame = '/odom'

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



    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def move_distance_using_odometry(self):

        # Initialize the position variable as a Point type
        xt, yt, zt= self.getkey()
        for i in range(len(xt)):
            position = Point()
            move_cmd = Twist()
            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed
            # Get the starting position values
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            (goal_x, goal_y, goal_z) = [float(xt[i]), float(yt[i]), float(zt[i])]
            goal_z = np.radians(goal_z)
            # Keep track of the distance traveled ,distance is the error for length, x,y
            distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            # Enter the loop to move along a side
            while distance < goal_distance and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                move_cmd.linear.x=linear_speed
                self.cmd_vel.publish(move_cmd)
                self.previouserror=error
                r.sleep()
                # Get the current position
                (position, rotation) = self.get_odom()
                # Compute the Euclidean distance from the start
                distance = sqrt(pow((position.x - x_start), 2) +
                                pow((position.y - y_start), 2))
                self.x_error.append(position.x - x_start)
                self.y_error.append(position.y - y_start)
                self.t_p.append(time.clock_gettime(3))
            print('\t** Completed linear motion of Step {} in Iteration {} **'.format(s, r))


            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            # Set the movement command to a rotation
            move_cmd.angular.z = angular_speed
            # Track the last angle measured
            last_angle = rotation
            # Track how far we have turned
            turn_angle = 0
            # Begin the rotation
            while abs(turn_angle + angular_tolerance) < abs(goal_z) and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                move_cmd.angular.z = angular_speed
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                # Get the current rotation
                (position, rotation) = self.get_odom()
                # Compute the amount of rotation since the last lopp
                delta_angle = normalize_angle(rotation - last_angle)
                turn_angle += delta_angle
                last_angle = rotation
            print('\t** Completed rotation motion of Step {} in Iteration {} **'.format(s, r))

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)



    def execute_plan(self):

        self.move_straight_for_distance_using_odometry()
        print( '------------- Plan execution completed! -------------')
        # Stop the robot when we are done
        self.cmd_vel.publish(Twist())

    def plottest(self):
        t=[]
        t=self.t_p.append(time.clock_gettime(3))
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

    def getkey(self):

        xt=[2,-2,-2, 2,2]
        yt=[3,3,-3,-3,0]
        zt=np.array([0,90,0,-90,0])

        return xt, yt, zt



    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)



def main():



    path=NavSquare()
    while not rospy.is_shutdown():
        #print('laser disturbuted')
        path=NavSquare()
        path.pathplot()
        show()
        r.sleep()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
