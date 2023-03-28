#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback1(data):
    print("I heard1",data.data)
def callback2(data):
    print("I heard2",data.data)
def callback(data):
    print("I heard0",data.data)

rospy.init_node('listener')
rospy.Subscriber("chatter", String, callback)
rospy.Subscriber("chatter1", String, callback1)
rospy.Subscriber("chatter2", String, callback2)
rospy.spin()
