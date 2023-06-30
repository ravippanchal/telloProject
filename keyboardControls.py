import keyboardModule as kp
from djitellopy import tello
from time import sleep
import cv2

kp.init()

me = tello.Tello()
me.connect(False)
print(me.get_battery)
me.streamon()


def getInputs():
    lr,fb,ud, mc = 0,0,0,0
    speed = 50
    if kp.getKeys("UP"): fb = speed
    elif kp.getKeys("DOWN"): fb = -speed

    if kp.getKeys("LEFT"): lr = -speed
    elif kp.getKeys("RIGHT"): lr = speed

    if kp.getKeys("w"): ud = speed
    elif kp.getKeys("s"): ud = -speed

    if kp.getKeys("d"): mc = speed
    elif kp.getKeys("a"): mc = -speed

    if kp.getKeys("q"): me.takeoff()
    if kp.getKeys("e"): me.land()
    
    return [lr,fb,ud,mc]

while True:
    inputs = getInputs()
    me.send_rc_control(inputs[0],inputs[1],inputs[2],inputs[3])
    img = me.get_frame_read().frame
    img = cv2.resize(img,(360,240))
    cv2.imshow("Image",img)
    sleep(0.05)