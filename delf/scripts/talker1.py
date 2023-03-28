#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter1', String, queue_size=10)
    rospy.init_node('talker1',anonymous=False)
    #rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(1) # 10hz
    hello_str = "hello world1 %s" 
    while not rospy.is_shutdown():
        rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
