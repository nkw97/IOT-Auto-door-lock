#! /usr/bin/python

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
from picamera import PiCamera
from picamera.array import PiRGBArray
import imutils
import pickle
import time
import dlib
import cv2
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from gpiozero import MotionSensor
from gpiozero import LED


# Setup Relay model and Selunoid Lock
RELAY = 17
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY, GPIO.OUT)
GPIO.output(RELAY,GPIO.LOW)
pir = MotionSensor(4)
red_led=LED(27)
red_led.off()



# Setup mqtt
mqttHost = 'localhost' 
mqttPort = 1883 
mqttUser = None
mqttPassword = None 
mqttClientId = "face_recognition_to_mqtt" #each client needs a unique ID
mqttTopicPrefix = "cmnd/faceToMqtt/user/" #mqtt message will be this plus detected users name appended to end
mqttPayload = "detected"

#set PIR sensor
def motion():
    while True:
        pir.wait_for_motion()
        red_led.on()
        pir.wait_for_no_motion()
        red_led.off()
        return True
    
if motion():
    print("motion")
    def recognizeToMQTT():
        #Initialize 'currentname' to trigger only when a new person is identified.
        currentname = "unknown"
        #Determine faces from encodings.pickle file model created from train_model.py
        encodingsP = "encodings.pickle"

        # load the known faces and embeddings along with OpenCV's Haar
        # cascade for face detection
        print("[INFO] loading encodings + face detector...")
        data = pickle.loads(open(encodingsP, "rb").read())


        vs = VideoStream(src=0,framerate=10).start()
        #vs = VideoStream(usePiCamera=True).start()
        time.sleep(2.0)

        # start the FPS counter
        fps = FPS().start()

        prevTime = 0
        doorUnlock = False

        # loop over frames from the video file stream
        while True:
            # grab the frame from the threaded video stream and resize it
            # to 500px (to speedup processing)
            frame = vs.read()
            frame = imutils.resize(frame, width=400)
            # Detect the fce boxes
            boxes = face_recognition.face_locations(frame)
            # compute the facial embeddings for each face bounding box
            encodings = face_recognition.face_encodings(frame, boxes)
            names = []

            # loop over the facial embeddings
            for encoding in encodings:
                # attempt to match each face in the input image to our known
                # encodings
                matches = face_recognition.compare_faces(data["encodings"],
                    encoding)
                name = "Unknown" #if face is not recognized, then print Unknown

                # check to see if we have found a match
                if True in matches:
                    # find the indexes of all matched faces then initialize a
                    # dictionary to count the total number of times each face
                    # was matched
                    matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                    counts = {}
                    
                    GPIO.output(RELAY,GPIO.HIGH)
                    prevTime = time.time()
                    doorUnlock = True
                    print("door unlock")

                    # loop over the matched indexes and maintain a count for
                    # each recognized face face
                    for i in matchedIdxs:
                        name = data["names"][i]
                        counts[name] = counts.get(name, 0) + 1

                    # determine the recognized face with the largest number
                    # of votes (note: in the event of an unlikely tie Python
                    # will select first entry in the dictionary)
                    name = max(counts, key=counts.get)

                    #If someone in your dataset is identified, print their name on the screen
                    if currentname != name:
                        currentname = name
                        print(currentname)

                # update the list of names
                names.append(name)
             #lock the door after 5 seconds
            if doorUnlock == True and time.time() - prevTime > 5:
                doorUnlock = False
                GPIO.output(RELAY,GPIO.LOW)
                print("door lock")
                #fps.stop()
                vs.stop()
            # loop over the recognized faces
            for ((top, right, bottom, left), name) in zip(boxes, names):
                # draw the predicted face name on the image - color is in BGR
                cv2.rectangle(frame, (left, top), (right, bottom),
                    (0, 255, 225), 2)
                y = top - 15 if top - 15 > 15 else top + 15
                cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                    .8, (0, 255, 255), 2)

            # display the image to our screen
            cv2.imshow("Facial Recognition is Running", frame)
            key = cv2.waitKey(1) & 0xFF

            # quit when 'q' key is pressed
            if key == ord("q"):
                break

            # update the FPS counter
            fps.update()

        # stop the timer and display FPS information
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

if __name__ == "__main__":
	
    print("\n\n\nFACE RECOGNITION TO MQTT - main face recognition")     
    print("* captures frames from picamera and compares to see if a known user/person is captured")    
    print("* fires off an mqtt message if a known user/person is detected \n\n\n")    
    
    
    recognizeToMQTT()
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()

