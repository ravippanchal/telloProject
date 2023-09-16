import cv2
import numpy as np
from djitellopy import tello
import time

# Initialize the Tello drone connection
drone = tello.Tello()

# Connect to the drone
drone.connect(False)

# Print drone's battery status
print(drone.get_battery())

# Start the drone's video stream
drone.streamon()

# Take off with the drone
drone.takeoff()

# Control the drone's movements (left/right, forward/backward, up/down, yaw)
drone.send_rc_control(0, 0, 27, 0)

# Pause for 2.2 seconds
time.sleep(2.2)

# Set default image width and height
w, h = 360, 240

# Define the range for the face's area to be in the frame
faceBoundRange = [6200, 6800]

# PID values for controlling the drone's movement
pidValues = [0.4, 0.4, 0]

# Initialize the previous error for PID control
previousError = 0

def detectFace(image):
    """Detect faces in the given image."""
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(grayImage, 1.2, 8)
    faceCenters = []
    faceAreas = []

    # Iterate over detected faces and mark them on the image
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(image, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        faceCenters.append([cx, cy])
        faceAreas.append(area)

    # Return the largest detected face's center and area
    if len(faceAreas) != 0:
        i = faceAreas.index(max(faceAreas))
        return image, [faceCenters[i], faceAreas[i]]
    else:
        return image, [[0, 0], 0]

def followFace(info, w, pidValues, previousError):
    """Control the drone to follow the detected face."""
    area = info[1]
    x, y = info[0]
    fb = 0
    error = x - w // 2
    speed = pidValues[0] * error + pidValues[1] * (error - previousError)
    speed = int(np.clip(speed, -100, 100))

    # Determine forward-backward movement based on face's area
    if area > faceBoundRange[0] and area < faceBoundRange[1]:
        fb = 0
    elif area > faceBoundRange[1]:
        fb = -20
    elif area < faceBoundRange[0] and area != 0:
        fb = 20
    if x == 0:
        speed = 0
        error = 0

    # Send control commands to the drone
    print(speed, fb)
    drone.send_rc_control(0, fb, 0, speed)

    return error

# Main loop to continuously detect and follow the face
while True:
    # Capture frame from drone's video stream
    image = drone.get_frame_read().frame
    image = cv2.resize(image, (w, h))
    image, info = detectFace(image)
    previousError = followFace(info, w, pidValues, previousError)

    # Display the processed image
    cv2.imshow("Output", image)
    
    # Check for 'q' key press to land the drone and exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        drone.land()
        break
