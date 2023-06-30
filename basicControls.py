
from djitellopy import Tello
from time import sleep

tello = Tello()
tello.connect(False)
tello.takeoff()
tello.send_rc_control(-10,0,0,0)
sleep(1)
tello.land()