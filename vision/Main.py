import cv2
import numpy as np
import robotpy_apriltag as apriltag
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging
logging.basicConfig(level=logging.DEBUG)

def main():
   tagSize = 0.17 # meters
   focalCenterX = 325.00253538
   focalCenterY = 235.65891798
   focalLengthX = 656.29804936
   focalLengthY = 655.66760244

   # Set up networktables
   ip = "10.54.13.2"
   NetworkTables.initialize(server=ip)
   smartDashboard = NetworkTables.getTable("SmartDashboard")

   # Initialize the camera
   cap = cv2.VideoCapture(0)

   # Create an AprilTag detector
   detector = apriltag.AprilTagDetector()
   detector.addFamily("tag36h11")
   config = apriltag.AprilTagPoseEstimator.Config(tagSize, focalLengthX, focalLengthY, focalCenterX, focalCenterY)
   estimator = apriltag.AprilTagPoseEstimator(config)

   while True:
      # Capture a frame from the camera
      ret, frame = cap.read()

      # if the cam read fails, break
      if not ret:
            break
      
      # Convert the frame to grayscale for AprilTag detection
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      
      # Detect AprilTags in the grayscale frame
      tags = detector.detect(gray)
      
      # Get tag id 1
      tagToFollow = next(filter(lambda x: x.getId() == 1, tags), None)

      # If the tag wasn't found, skip the rest of the loop
      if tagToFollow is None:
         continue

      pose = estimator.estimate(tagToFollow)

      x, y, zRot = pose.X, pose.Y, pose.rotation.Z
         
      smartDashboard.putNumberArray("transform", [x, y, zRot])
         
   
   # Release the camera and close OpenCV windows
   cap.release()
   cv2.destroyAllWindows()

if __name__ == "__main__":
   main()
