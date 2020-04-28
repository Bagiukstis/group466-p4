#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 09:44:14 2020

@author: frederik
"""

#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('flick2', Bool, queue_size=1)
    rospy.init_node('flick2', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        flickering = 1
        pub.publish(flickering)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
