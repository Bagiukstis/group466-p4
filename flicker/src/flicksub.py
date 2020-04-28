#!/usr/bin/env python
import rospy
#import video_stream_opencv
import cv2
import sys
import numpy as np
from std_msgs.msg import Bool

flick = 0
flick2 = 0
flick3 = 0

def callback(data):
    #rospy.loginfo("I heard: %s", data.data)
    global flick
    flick = 1
    
def callback2(data):
    #rospy.loginfo("I heard: %s", data.data)
    global flick2
    flick2 = 1


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("flick", Bool, callback)
    rospy.Subscriber("flick2", Bool, callback2)
    
    
    cap = cv2.VideoCapture(2)
    
    if (cap.isOpened() is False):
        print("Failed to initialize capture from webcam. Exiting... \n")
        sys.exit(0)
    
    
    print("Press 'q' to exit!")
    # Main loop
    while (True):
        # Read image from webcam
        ret, frame = cap.read()
        if (ret is False):
            break 
    
        global flick
        if(flick):
            #rospy.loginfo("Flicked")
            cv2.putText(frame, '10Hz', (180,270), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,0,255), 10, cv2.LINE_AA) 
            flick = 0
            
        global flick2
        if(flick2):
            #rospy.loginfo("Flicked")
            cv2.putText(frame, '5Hz', (180,400), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,0,255), 10, cv2.LINE_AA) 
            flick2 = 0
    
        
        cv2.imshow("Feed", frame)
        # Quit program is 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

