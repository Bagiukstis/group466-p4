#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  4 15:24:52 2020

@author: frederik

SSVEP Flicker Activity extractor

Takes a 240fps recording, and extracts the activity of blinking red light
"""
import cv2
import numpy as np

#Creating file for saving activity
saveFile = open("activity.txt","w")


cap = cv2.VideoCapture("source.mp4")

f = 0

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
    #Getting a binary image of the activity.
    binary = cv2.inRange(frame, (0, 0, 200), (100, 255, 255))
    # Save the white pixel count
    count = str(cv2.countNonZero(binary))
    f +=1 #Counts which frame and thus the time.
    saveFile.write(count + "," + str(f) +"/240" "\n")
    
    # Display the resulting frame
    cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)   # Create window with free diemnsions
    cv2.imshow('Frame',frame)
    cv2.namedWindow("Binary", cv2.WINDOW_NORMAL)   # Create window with free diemnsions
    cv2.imshow('Binary',binary)
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  # Break the loop
  else: 
    break
# When everything done, release the video capture object
cap.release()
# Closes all the frames
cv2.destroyAllWindows()
saveFile.close()
