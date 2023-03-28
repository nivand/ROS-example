#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rospy
import time
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    hello_str ="set your goal"
    t4=rospy.get_time()
    while not rospy.is_shutdown():
        rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        print(hello_str)
        t1=rospy.Time.now() #get time as rospy.Time instance
        t2=rospy.get_rostime() #get time as rospy.Time instance, same as .now()
        t3=rospy.get_time() #get time as float secs
        print('now',t1.to_sec())
        #print('rospy.get_rostime',t2.to_sec(),t2.secs, t2.nsecs)
        dt=t3-t4
        print('rospy.get_time',dt)
        t4=t3
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
