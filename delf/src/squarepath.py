#! /usr/bin/env python3

# to be executed with:
# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

# Import the Python library for ROS
import rospy
import time

# Import the Twist message
from geometry_msgs.msg import Twist

# Import the Odometry message
from nav_msgs.msg import Odometry

import tf

import numpy as np
from math import radians, copysign, sqrt, pow, pi, atan2
from std_msgs.msg import Float32MultiArray

import sys

# for the ground truth
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
# http://docs.ros.org/api/gazebo_msgs/html/srv/GetModelState.html
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi
import PyKDL
from math import pi

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


class MoveOpenLoop():

    def __init__(self, plan = []):
        rospy.init_node('squarepath', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Set a publish velocity rate of in Hz
        rate = rospy.get_param('/rate', 200)
        self.rate = rospy.Rate(rate)

        # this must correspond to what the spawn() service has generated in gazebo
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        print('Robot model:', self.robot_model)

        #print 'Starting ground truth service ...'
        #self.start_ground_truth_service()

        # get the pose from the odometry
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        self.odom_sub = rospy.Subscriber('/odom', Odometry,
                                         self.callback_odometry, queue_size=1)

        # get velocities to be used for moving the robot
        self.v = rospy.get_param('~lin_velocity', 0.3)
        self.omega = rospy.get_param('~ang_velocity', 0.3)

        print('Robot velocity: ({:.2f}, {:.2f})'.format(self.v, self.omega))

        # Creates a var of msg type Twist for velocity
        self.velocity = Twist()

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

        #pid initialize
        #self.kp=0.12
        #self.ki=0.0
        #self.kd=0.0010

        self.kp=0.12
        self.ki=0.4
        self.kd=0

        # errorparameter
        self.errorsum=0.0
        self.previouserror=0.0
        self.errorderivative=0.0


        # initialize all twist velocities to zero
        self.velocity.linear.x =  0.0
        self.velocity.linear.y =  0.0
        self.velocity.linear.z =  0.0
        self.velocity.angular.x =  0.0
        self.velocity.angular.y =  0.0
        self.velocity.angular.z =  0.0

        #self.reset_odo = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        # pose variables
        self.x = -999
        self.y = -999
        self.yaw = -999

        # wait for an input from the odometry topic to be sure that pose is correctly set when start
        print ('Wait for first odometry data...',)
        while ((self.x == -999) or (self.y == -999) or (self.yaw == -999)):
            self.rate.sleep()
        print(' done!')

        print ("Initial Pose: ({:.2f}, {:.2f}) {:.2f}".format(self.x, self.y, self.yaw))

        # this is the plan to actuate, as a list of translations and rotations
        self.plan = plan[:]


    # def update_pose_with_ground_truth(self):
    #     self.x, self.y = self.get_position_ground_truth(self.robot_model)
    #     siny, cosy = self.get_orientation_ground_truth(self.robot_model)
    #     self.yaw = np.arctan2(siny, cosy)

    def callback_odometry(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_euler(msg)

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.odo_angle_y = 2.0 * (q[3] * q[2] + q[0] * q[1]) # siny
        self.odo_angle_x = 1.0 - 2.0 * (q[1]**2 + q[2]**2) # cosy


    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        #print "Roll: %5.2f  Pitch: %5.2f  Yaw: %5.2f" % (roll, pitch, yaw)
        return yaw


    # compute the straight line distance between two points in 2D
    def euclidean_distance(self, u, v):
        return np.sqrt( (u[0] - v[0])*(u[0] - v[0]) + (u[1] - v[1])*(u[1] - v[1]) )


    # compute the difference between two angles according to atan2() quadrant rules
    def angle_distance(self, a1, a2):
        #atan2(y1, x1) +- atan2(y2, x2) = atan2(y1*x2 +- y2*x1,  x1 * x2 -+ y1*y2)
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)

        sum_a1_a2 = np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)

        print('Angle distance: {:.3f}'.format(sum_a1_a2))

        return sum_a1_a2


    def move_straight_for_distance_using_odometry(self,targetgoal):
        self.start_xy = (self.x, self.y)
        self.velocity.linear.x = self.v
        self.velocity.angular.z = 0.0
        tetapast=0.0
        a=self.euclidean_distance(self.start_xy,(targetgoal[0], targetgoal[1]))


        while self.euclidean_distance(self.start_xy,(self.x, self.y)) < a:
            (position, rotation) = self.get_odom()
            error=self.euclidean_distance(targetgoal[0:2],(position.x, position.y))
            print('error',error)
            dt=1/200
            self.errorderivative=(error-self.previouserror)/dt
            self.errorsum += error*dt
            self.velocity.linear.x=(self.kp*error)+(self.ki*self.errorsum)+(self.kd*self.errorderivative)
            self.previouserror=error
            print(self.velocity.linear.x)
            #if self.velocity.linear.x > 0.3:
            #    self.velocity.linear.x=0.3
            #Teta=arctan2((position.x-targetgoal[1])/(position.x-targetgoal[0]))
            #Teta=atan2((position.y-self.start_xy[1])/(position.x-self.start_xy[0]))
            self.vel_pub.publish(self.velocity)
            self.rate.sleep()


    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def rotate_for_angle_using_odometry(self, target_rotation):
        self.start_yaw = self.yaw

        if target_rotation < 0:
            self.velocity.angular.z = -self.omega
        else:
            self.velocity.angular.z = self.omega

        while abs(self.angle_distance(self.start_yaw, self.yaw)) <= abs(target_rotation):
            self.vel_pub.publish(self.velocity)
            self.rate.sleep()


    def execute_plan(self, repeat = 1):

        for r in range(repeat):
            print('------------- Starting iteration {} -------------'.format(r))

            for s, step in enumerate(plan):

                # first part of a plan step consists in moving for a given distance
                target_distance = step[0]
                targetgoal=step[0:3]
                self.move_straight_for_distance_using_odometry(targetgoal)

                # set the linear velocity to zero
                self.velocity.linear.x = 0.0
                self.vel_pub.publish(self.velocity)

                print('\t** Completed linear motion of Step {} in Iteration {} **'.format(s, r))


                # second part of a plan step consists in rotating for a given angle
                target_rotation = np.radians(step[2])
                self.rotate_for_angle_using_odometry(target_rotation)

                self.velocity.angular.z = 0.0
                self.vel_pub.publish(self.velocity)

                print('\t** Completed rotation motion of Step {} in Iteration {} **'.format(s, r))

            print( '------------- Plan execution completed! -------------')


     # Ensure a smooth shutdown of the robot
    def shutdown(self):

        rospy.loginfo("**** Stopping TurtleBot! ****")

        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

        self.vel_pub.publish(self.velocity)

        rospy.sleep(1)


if __name__ == '__main__':

    # This is a plan for a square motion
    plan = [(2,0,90),(2,3,90)]
    repeat = 1

    motion_controller = MoveOpenLoop(plan)

    motion_controller.execute_plan(repeat)

#except:
#    rospy.loginfo("node terminated")
