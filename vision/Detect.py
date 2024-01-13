import cv2
import numpy as np
import robotpy_apriltag as apriltag
import os
import time

def main():
    tagSize = 0.17 # Tag size for competition is ~17 cm
    focalCenterX = 0 # Currently unknown, pixels
    focalCenterY = 0 # Currently unknown, pixels
    focalLengthX = 0 # Currently unknown, pixels
    focalLengthY = 0 # Currently unknown, pixels

    # Initialize the camera
    cap = cv2.VideoCapture(0)  # Use the camera at index 0 (usually the built-in camera)


    # Create an AprilTag detector
    detector = apriltag.AprilTagDetector()
    detector.addFamily("tag36h11")
    #config = apriltag.AprilTagPoseEstimator.Config(tagSize, focalLengthX, focalLengthY, focalCenterX, focalCenterY)
    #estimator = apriltag.AprilTagPoseEstimator(config)
    prevId = False

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
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
            #pose = estimator.estimate(tag)

            if prevId == tag_id:
                #os.system('clear')
                continue
            else:
                os.system('cls' if os.name=='nt' else 'clear') # clears the console window
                os.system(f"notify-send \"April tag detected: ID = {tag_id}\"" if os.name!='nt' else '') # Sends unix graphical notification if possible
                print(f"Tag ID: {tag_id}") # Displays the tag_id in the console

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
