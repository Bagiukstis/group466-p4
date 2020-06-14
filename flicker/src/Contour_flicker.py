#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 14 12:13:34 2020

@author: frederik
"""

import rospy
#import video_stream_opencv
import cv2
import sys
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

init = 1
background = np.zeros(( 420, 69,3), np.uint8)
img = np.ones(( 69, 69,3), np.uint8)
img2 = np.ones(( 69, 420,3), np.uint8)

def callback(data):
    #rospy.loginfo("I heard: %s", data.data)
    global flick
    flick = 1
    
def callback2(data):
    #rospy.loginfo("I heard: %s", data.data)
    global flick2
    flick2 = 1


def callback3(data):
    bridge = CvBridge()
    try:
        global img
        img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        # Stuff here
    except CvBridgeError:
        print(CvBridgeError)
        
    global init
    if(init):
        global background
        background = np.zeros((img.shape[0],img.shape[1],3), np.uint8)
        init = 0
        
def callback4(data):
    bridge = CvBridge()
    try:
        global img2
        img2 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        # Stuff here
    except CvBridgeError:
        print(CvBridgeError)
        
    global init
    if(init):
        global img, background
        background = np.zeros((img2.shape[0],img2.shape[1],3), np.uint8)
        init = 0
    
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("flick", Bool, callback)
    rospy.Subscriber("flick2", Bool, callback2)
    
    rospy.Subscriber("cv/contours", Image, callback3)
    rospy.Subscriber("cv/contours2", Image, callback4)
    
    
    cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)   # Create window with free diemnsions
    
    # Main loop
    while (True):
        global background
        cv2.resizeWindow("Frame", background.shape[0], background.shape[1]) #Resizes the window to fit the image displayed.
        cv2.imshow("Frame", background)
        
        global flick
        if(flick):
            global img
            cv2.resizeWindow("Frame", img.shape[0], img.shape[1]) #Resizes the window to fit the image displayed.
            cv2.imshow("Frame",img)
            flick = 0
            
        global flick2
        if(flick2):
            global img2
            cv2.resizeWindow("Frame", img2.shape[0], img2.shape[1]) #Resizes the window to fit the image displayed.
            cv2.imshow("Frame",img2)
            flick2 = 0
            
        # Quit program is 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up
    cv2.destroyAllWindows()
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

