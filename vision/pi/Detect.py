import cv2
import numpy as np
import robotpy_apriltag as apriltag
import os
import time
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

def main():
    tagSize = 0.17 # Tag size for competition is ~17 cm
    focalCenterX = 325.00253538 # Currently unknown, pixels
    focalCenterY = 235.65891798 # Currently unknown, pixels
    focalLengthX = 656.29804936 # Currently unknown, pixels
    focalLengthY = 655.66760244 # Currently unknown, pixels

    # Experimental CameraServer Init Code
    #CameraServer.enableLogging()

    #
    #refrence = UsbCamera()
    #cap = cv2.VideoCapture(0)
    #camera = CameraServer.startAutomaticCapture("Stellar Sight", "/dev/video0")
    #camera.setResolution(1280, 720)
    #cvSink = CameraServer.getVideo()
    #output_stream = CameraServer.putVideo('Processed', 1280, 720)

    #input_img = np.zeros(shape=(1280, 720, 3), dtype=np.uint8)

    #time, input_img = cvSink.grabFrame(input_img)

    #if input_img.any():
        #print("I equal something!!!!!!!")
   # else:
       # print("I am nothing!!!!")

    #if time == 0:
        #print("We Error'd Out!")
        #print(cvSink.getError())
        
    

    # Initialize the camera
    cap = cv2.VideoCapture(1)


    # Create an AprilTag detector
    detector = apriltag.AprilTagDetector()
    detector.addFamily("tag36h11")
    config = apriltag.AprilTagPoseEstimator.Config(tagSize, focalLengthX, focalLengthY, focalCenterX, focalCenterY)
    estimator = apriltag.AprilTagPoseEstimator(config)
    prevId = False

    #input_img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()

        #time, input_img = cvSink.grabFrame(input_img)

        
        if not ret:
            #print(cvSink.getError())
            break
        
        # Convert the frame to grayscale for AprilTag detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags in the grayscale frame
        tags = detector.detect(gray)
        
        # Iterate through detected tags
        for tag in tags:
            # Draw the tag outline and ID on the frame
            # tag.draw(frame)
            
            # Extract tag data
            tag_id = tag.getId()
            tag_family = tag.getFamily()
            pose = estimator.estimate(tag)


            os.system('cls' if os.name=='nt' else 'clear') # clears the console window
            #os.system(f"notify-send \"April tag detected: ID = {tag_id}\"\n{pose}" if os.name!='nt' else '') # Sends unix graphical notification if possible
            print(f"Tag ID: {tag_id}\n{pose}") # Displays the tag_id in the console

            prevId = tag_id
        
        # Display the processed frame with detected tags
        cv2.imshow('AprilTags', frame)

        
        # Exit when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the camera and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
