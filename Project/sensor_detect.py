#! /usr/bin/python

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
from picamera import PiCamera
import face_recognition
import imutils
import pickle
import time
import cv2
from gpiozero import MotionSensor
from gpiozero import LED


pir = MotionSensor(4)
camera = PiCamera()
camera.rotation = 180
blue_led=LED(17)
blue_led.off()

currentname = "unknown"
encodingsP = "encodings.pickle"
print("[INFO] loading encodings + face detector...")
data = pickle.loads(open(encodingsP, "rb").read())
vs = VideoStream(src=0,framerate=10).start()
#vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
fps = FPS().start()

def motion():
     while True:
        pir.wait_for_motion()
        return True

def camera():
    
    start_time = time.time()
    while ((time.time() - start_time) < 10.1):
        ret, frame = video_capture.read()
    



    



