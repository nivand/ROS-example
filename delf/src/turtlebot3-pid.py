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
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
kp_distance = 10
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
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

        (xt, yt, zt) = self.getkey()
        for i in range(len(xt)):

            (goal_x, goal_y, goal_z) = [float(xt[i]), float(yt[i]), float(zt[i])]
            if goal_z > 180 or goal_z < -180:
                print("you input wrong z range.")
                self.shutdown()

            goal_z = np.deg2rad(goal_z)

            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            #distance is the error for length, x,y
            distance = goal_distance
            previous_distance = 0
            total_distance = 0

            previous_angle = 0
            total_angle = 0


            while distance > 0.05:
                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y
                #path_angle = error
                path_angle = atan2(goal_y - y_start, goal_x- x_start) - rotation

                if path_angle < -pi/4 or path_angle > pi/4:
                    if goal_y < 0 and y_start < goal_y:
                        path_angle = -2*pi + path_angle
                    elif goal_y >= 0 and y_start > goal_y:
                        path_angle = 2*pi + path_angle
                if last_rotation > pi-0.1 and rotation <= 0:
                    rotation = 2*pi + rotation
                elif last_rotation < -pi+0.1 and rotation > 0:
                    rotation = -2*pi + rotation


                diff_angle = path_angle - previous_angle
                diff_distance = distance - previous_distance

                distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

                control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

                control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

                move_cmd.angular.z = (control_signal_angle) - rotation
                #move_cmd.linear.x = min(linear_speed * distance, 0.1)
                move_cmd.linear.x = min(control_signal_distance, 0.1)

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                last_rotation = rotation
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                previous_distance = distance
                total_distance = total_distance + distance
                print("Current positin and rotation are: ", (position, rotation))

            # rospy.loginfo("Stopping the robot...")

        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("reached :)   ^_^")

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            self.cmd_vel.publish(Twist())
            return

    def getkey(self):

        xt,yt,zt=[],[],[]
        xt=[2,2,-2,-2, 2,2]
        yt=[0,3,3,-3,-3,0]
        zt=np.array([90,0,90,0,-90,0])

        a = 0.17
        k = math.tan(a)
        xt , yt, zt = [] , [],[]
        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            dz=atan2(dy,dx)
            xt.append(dx)
            yt.append(dy)
            zt.append(dz)



        return xt, yt, zt

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


if __name__ == '__main__':


    print('Initial starting angle Theta wrt +X axis: ')

    # caller = 'roslaunch control_bot gazebo_user.launch x_pos:={0} y_pos:={1} z_pos:={2}'.format(str(initial_position[0]), str(initial_position[1]), str(0))

    while not rospy.is_shutdown():
        GotoPoint()
