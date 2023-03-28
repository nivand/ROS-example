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

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

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
        rospy.init_node('turtlebot3_path_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.reached=False
        self.i=0

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
        linear_speed = 1    #kp_distance
        angular_speed = 1  #kp_angular


        #xt=np.array([1,1,1,1,-3])
        #yt=np.array([2,2,-2,-2,2])
        #zt=np.array([0,90,0,-90,0])



        #xt=[2,-2,-2, 2,2]
        #yt=[3,3,-3,-3,0]
        #zt=np.array([0,90,0,-90,0])



        xt , yt, zt = [] , [],[]



        r2= 2
        t = np.linspace( 0 ,  np.pi , 4 )
        X2 = r2 * np.cos(t)
        Y2 = r2* np.sin(t)
        Z2=t
        
        #.......
        R2 = 6
        t = np.linspace( np.pi,np.pi*2, 4 )
        X4 = R2* np.cos(t)
        Y4 = R2 * np.sin(t)
        Z4=t
        xt.append(X4)
        yt.append(Y4)
        zt.append(Z4)




        for j in range(len(X2)):
            for i in range(len(xt)):
                (goal_x, goal_y, goal_z) = [float(xt[j][i]), float(yt[j][i]), float(zt[j][i])]
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
                    path_angle = atan2(goal_y - y_start, goal_x- x_start)- rotation
                    path_angle=normalize_angle(path_angle)



                    diff_angle = path_angle - previous_angle
                    diff_distance = distance - previous_distance

                    distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

                    control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

                    control_signal_angle = kp_angle*path_angle

                    move_cmd.angular.z = (control_signal_angle)
                    #move_cmd.linear.x = min(linear_speed * distance, 0.1)
                    move_cmd.linear.x = min(control_signal_distance, 0.2)

                    if move_cmd.angular.z > 0:
                        move_cmd.angular.z = min(move_cmd.angular.z, 1.1)
                    else:
                        move_cmd.angular.z = max(move_cmd.angular.z, -1.1)

                    last_rotation = rotation
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                    previous_distance = distance
                    total_distance = total_distance + distance
                    print("Current positin and rotation are: ", (position, rotation))



        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("reached :)   ^_^")
        self.reached=False
        self.i+=1
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


        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        return

    def getkey(self):
        global x_input, y_input, z_input

        X1 = np.linspace(-3, 3 , 100)
        Y1 = np.array([2]*100)
        Y2 = np.linspace(2, -2 , 100)
        X2 = np.array([3]*100)
        X3 = np.linspace(3, -3 , 100)
        Y3 = np.array([-2]*100)
        Y4 = np.linspace(-2, 2 , 100)
        X4 = np.array([-3]*100)


        return x, y, z





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




#print("Please enter the lower range for generating x and y coordinates for the starting position of Turtlebot")
#lower = input()

#print("Please enter the upper range for generating x and y coordinates for the starting position of Turtlebot")
#upper = input()

#print("Random initial X and Y coordinates of the Turtlebot are:-")

#coord = np.random.uniform(lower, upper, 2)
#coord=np.array([2,3])
#print('Initial X coordinate: ', coord[0])
#print('Initial Y coordinate: ', coord[1])

#print("Please enter the lower range for generating angle wrt x axis for the starting position of Turtlebot in degrees")
#lower_angle = math.radians(input())

#print("Please enter the upper range for generating angle wrt x axis for the starting position of Turtlebot in degrees")
#upper_angle = math.radians(input())

#angle = np.random.uniform(lower_angle, upper_angle, 1)
#angle=[90]
#print('Initial starting angle Theta wrt +X axis: ', angle[0])

#initial_position = coord + angle
#initial_position = np.concatenate((coord,angle))

initial_position=np.array([0, 0, 0])

#print('(X, Y, Theta):' ,coord[0], coord[1], angle[0])
print('Initial pose is:-')
print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])

#print("Enter final x position")
#x_final = input()
#print("Enter final y position")
#y_final = input()
#print("Enter final angle position")
#angle_final = input()

final = [0, 0, 0]
final_position = np.array(final)

x_input = final_position[0]
y_input = final_position[1]
z_input = final_position[2]


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

while not rospy.is_shutdown():
    GotoPoint()
