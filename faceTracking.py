import cv2
import numpy as np


def findFace(img):

    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)

    myFaceListC = []

    myFaceListArea = []

    for (x, y, w, h) in faces:

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
capture = cv2.VideoCapture(0)


while True:
    _,img = capture.read()
    findFace(img)
    cv2.imshow("Output",img)
    #findFace(img)
    cv2.waitKey(1)
