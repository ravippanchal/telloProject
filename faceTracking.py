import cv2
import numpy as np

capture = cv2.VideoCapture(0)

while True:
    _,img = capture.read()
    cv2.imshow("Output",img)
    cv2.waitKey(1)