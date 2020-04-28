#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('flick', Bool, queue_size=1)
    rospy.init_node('flick', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        flickering = 1
        pub.publish(flickering)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
