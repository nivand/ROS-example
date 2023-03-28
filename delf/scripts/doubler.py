#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

rospy.init_node('doubler')
double=0;
def callback(msg):
    #bad az moshakhas shodan topic doubled azash estefade shode
    #double=input('set')
    pub.publish(double)
    count +=1
    doubled=msg.data*2
    print doubled


sub=rospy.Subscriber("double", Int32,callback)
pub=rospy.Publisher('double', Int32,queue_size=10)
rospy.spin()
