#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


rospy.init_node('numbpublisher', anonymous=True)
pub = rospy.Publisher('counter', Int32, queue_size=10)
rate = rospy.Rate(2) # 10hz
count=0
while not rospy.is_shutdown():
    pub.publish(count)
    count +=1
    rate.sleep()
