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


#kp_distance = 10
#ki_distance = 0.01
#kd_distance = 0.5



kp_distance = 1
ki_distance = 10
kd_distance = 10

kp_angle = 1.2
#ki_angle = 0.5
#kd_angle = 0.4

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(20)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.reached=False
        self.i=0
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
        linear_speed = 1    #kp_distance
        angular_speed = 1  #kp_angular
        xt , yt, zt = [] , [],[]
        self.x_error=[]
        self.y_error=[]






        #zt=np.array([0,15,30,45,60,90,60,45,30,15,0,0,15,30,45,60,90,60,45,30,15,0])

        #xt=np.array([0, -6* np.sin(np.pi/12), -6* np.sin(np.pi/6), -6* np.sin(np.pi/4), -6* np.sin(np.pi/3), -6* np.sin(np.pi/2), -6* np.sin(np.pi/3),-6* np.sin(np.pi/4),-6* np.sin(np.pi/6),-6* np.sin(np.pi/12),0,0,2* np.sin(np.pi/12), 2* np.sin(np.pi/6), 2* np.sin(np.pi/4), 2* np.sin(np.pi/3), 2* np.sin(np.pi/2), 2* np.sin(np.pi/3),2*np.sin(np.pi/4),2* np.sin(np.pi/6),2* np.sin(np.pi/12),0])
        #yt=np.array([6, 6* np.cos(np.pi/12),6* np.cos(np.pi/6),6* np.cos(np.pi/4),6* np.cos(np.pi/3),6* np.cos(np.pi/2), -6* np.cos(np.pi/3),-6* np.cos(np.pi/4),-6* np.cos(np.pi/6),-6* np.cos(np.pi/12),-6,-2,-2* np.cos(np.pi/12),-2* np.cos(np.pi/6),-2* np.cos(np.pi/4),-2* np.cos(np.pi/3),-2* np.cos(np.pi/2), 2* np.cos(np.pi/3),2* np.cos(np.pi/4),2* np.cos(np.pi/6),2* np.cos(np.pi/12),2])

        #xt, yt, zt = self.getkey()




        growth_factor = 0.5
        zt=[]
        for i in range(400):
            t = i / 20 * math.pi
            dx = (1 + growth_factor * t) * math.cos(t)
            dy = (1 + growth_factor * t) * math.sin(t)
            xt.append(dx)
            yt.append(dy)
            dz=atan2(dy,dx)
            zt.append(t)

        #xt=[2,2,-2,-2, 2,2]
        #yt=[0,3,3,-3,-3,0]
        xt,yt,zt=self.getkey()

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


            while distance > 0.05 :
                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y
                #path_angle = error
                path_angle = atan2(goal_y - y_start, goal_x- x_start)- rotation
                path_angle=normalize_angle(path_angle)



                diff_angle = path_angle - previous_angle
                diff_distance = distance - previous_distance

                distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))


                control_signal_distance = kp_distance*distance + ki_distance*total_distance  + kd_distance*diff_distance

                control_signal_angle = kp_angle*path_angle

                move_cmd.angular.z = (control_signal_angle)
                #move_cmd.linear.x = min(linear_speed * distance, 0.1)
                move_cmd.linear.x = min(control_signal_distance, 0.4)

                #if move_cmd.linear.x> 0.9 :
                    #move_cmd.linear.x = 0.8


                self.x_error.append(goal_x - x_start)
                self.y_error.append(goal_y - y_start)

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
        self.reached=True
        self.i+=1
        while abs(rotation - goal_z) > 0.05 :
            (position, rotation) = self.get_odom()
            move_cmd.linear.x = 0.00
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())


    def plot_error(self):


        t.append(time.clock_gettime(3))
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



    def getkey(self):

        #path array

        # rectangular
        xt,yt,zt=[],[],[]
        i=20
        xt=np.concatenate([np.linspace(0,2,i),np.array([2]*i),np.linspace(2,-2,i), np.array([-2]*i),np.linspace(-2,2,i),np.array([2]*i)])
        yt=np.concatenate([np.array([0]*i),np.linspace(3,3,i),np.array([3]*i),np.linspace(-3,3,i),np.array([-3]*i),np.linspace(-3,0,i)])
        zt=np.array([0]*i*6)

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
            zt.append(0.17)


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

initial_position=np.array([0,0, 0])

#print('(X, Y, Theta):' ,coord[0], coord[1], angle[0])
print('Initial pose is:-')
print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])

#print("Enter final x position")
#x_final = input()
#print("Enter final y position")
#y_final = input()
#print("Enter final angle position")
#angle_final = input()

final = [2,0,0]
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

while not rospy.is_shutdown() :
    GotoPoint()
    t.append(time.clock_gettime(3))
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
