#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# #
rospy.init_node("talker")
pub=rospy.Publisher("chatter",String,queue_size=10)
rate=rospy.Rate(10)#10hz
# #
hello_str="hello world"
# #
while not rospy.is_shutdown():
    rospy.loginfo(hello_str)
    rospy.get_time()
    pub.publish(hello_str)
    rate.sleep()
