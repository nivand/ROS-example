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
import tf
from math import radians, copysign, sqrt, pow, pi
import PyKDL
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import subprocess
import os
import time
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

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
        rospy.init_node('squarefullpath', anonymous=False)
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        # How fast will we check the odometry values?
        rate = 100
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)
        # Set the parameters for the target square
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians
        print('linear from user')

        self.linear_speed = rospy.get_param("~linear_speed",0.4)        # meters per second
        self.angular_speed = rospy.get_param("~angular_speed", 0.2)      # radians per second
        self.angular_tolerance = radians(rospy.get_param("~angular_tolerance", 1)) # degrees to radians

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        #path
        self.path = Path()
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size=10)


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
        msg = rospy.wait_for_message("/odom" , Odometry)
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

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
    def move_straight_for_distance_using_odometry(self,targetgoal):

        position = Point()
        move_cmd = Twist()
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        # Get the starting position values
        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y
        # Keep track of the distance traveled ,distance is the error for length, x,y
        distance = 0
        while distance < targetgoal and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            move_cmd.linear.x=self.linear_speed
            self.cmd_vel.publish(move_cmd)

            # Get the current position
            (position, rotation) = self.get_odom()
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))


        self.x_error.append(position.x - x_start)
        self.y_error.append(position.y - y_start)
        self.t_p.append(time.clock_gettime(3))
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)


    def rotate_for_angle_using_odometry(self,target_rotation):
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed
        position = Point()
        # Get the starting position values
        (position, rotation) = self.get_odom()
        # Track the last angle measured
        last_angle = rotation
        # Track how far we have turned
        turn_angle = 0
        # Bgin the rotation
        while abs(turn_angle ) < abs(target_rotation) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            move_cmd.angular.z = self.angular_speed
            self.cmd_vel.publish(move_cmd)

            # Get the current rotation
            (position, rotation) = self.get_odom()
            # Compute the amount of rotation since the last lopp
            delta_angle = normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation


    def path_following(self):
        plan = [(2,90),(3,90),(4,90),(6,90),(4,90),(3,0)]
        #plan = [(2,90),(3,90)]

        for s, step in enumerate(plan):
                # first part of a plan step consists in moving for a given distance
                targetgoal = step[0]
                self.move_straight_for_distance_using_odometry(targetgoal)
                # set the linear velocity to zero

                #print('\t** Completed linear motion of Step {} in Iteration {} **'.format(s, r))
                # second part of a plan step consists in rotating for a given angle
                target_rotation = np.radians(step[1])
                self.rotate_for_angle_using_odometry(target_rotation)

                move_cmd = Twist()
                self.cmd_vel.publish(move_cmd)


                #print('\t** Completed rotation motion of Step {} in Iteration {} **'.format(s, r))
        self.reached = True



    def getkey(self):

        xt=np.array([2,2,-2,-2, 2,2])
        yt=np.array([0,3,3,-3,-3,0])
        zt=np.array([0,0,90,0,-90,0])

        return xt, yt, zt

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


def main():

    path=NavSquare()
    path.path_following()
    path.pathplot()
    show()
    r.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
