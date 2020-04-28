#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 13:40:32 2020

@author: frederik
"""

import cv2

locator = 0

for x in range(255):
    cap = cv2.VideoCapture(locator)

    if (cap.isOpened() is True):
        print("Found camera %s", locator)
        locator += 1
    
    if (cap.isOpened() is False):
        print("Did not find camera %s", locator)
        locator += 1
    
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

cap.release()

cv2.destroyAllWindows()