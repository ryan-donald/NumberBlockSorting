#! /usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #red
    low_red = np.array([161,155,84])
    high_red = np.array([179,255,255])
    mask = cv2.inRange(hsv_frame, low_red, high_red)

    cv2.imshow("Frame", frame)
    cv2.imshow("Red Mask", mask)

    key = cv2.waitKey(1)
    if key == 27:
        break
