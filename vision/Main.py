import cv2
import numpy as np
import robotpy_apriltag as apriltag
import ntcore

# To see messages from networktables, you must setup logging
#import logging
#logging.basicConfig(level=logging.DEBUG)

def calculateAbsoluteRobotPose(fieldLayout : apriltag.AprilTagFieldLayout, tagPose, tagId : int):
    absoluteTagPose = fieldLayout.getTagPose(tagId)
    return absoluteTagPose.transformBy(tagPose)
    

def main():
   tagSize = 0.17 # meters
   focalCenterX = 325.00253538
   focalCenterY = 235.65891798
   focalLengthX = 656.29804936
   focalLengthY = 655.66760244

   # Set up networktables
   #ip = "10.54.13.2"

   inst = ntcore.NetworkTableInstance.getDefault()
   inst.setServerTeam(5413)
   inst.startClient4("raspberrypi")

   smartDashboard = inst.getTable("SmartDashboard")
   xTopic = smartDashboard.getDoubleTopic("x").publish()
   zTopic = smartDashboard.getDoubleTopic("z").publish()
   rotTopic = smartDashboard.getDoubleTopic("rot").publish()

   absolutePoseTopic = smartDashboard.getDoubleArrayTopic("robotPose").publish()

   # Quinn trying to figure out how networktables works
   #piTable = NetworkTables.getTable("RaspberryPi")
   
   # Initialize the camera
   cap = cv2.VideoCapture(0)

   # Create an AprilTag detector
   detector = apriltag.AprilTagDetector()
   detector.addFamily("tag36h11")
   config = apriltag.AprilTagPoseEstimator.Config(tagSize, focalLengthX, focalLengthY, focalCenterX, focalCenterY)
   estimator = apriltag.AprilTagPoseEstimator(config)
   field = apriltag.AprilTagField.k2024Crescendo
   fieldLayout = apriltag.loadAprilTagLayoutField(field=field)

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
      tagToFollow = next(filter(lambda x: x.getId() == 16, tags), None)

      # If the tag wasn't found, skip the rest of the loop
      if tagToFollow is None:
         continue
      
      
      pose = estimator.estimate(tagToFollow)
      x, y, z, rotation = pose.X(), pose.Y(), pose.Z(), pose.rotation().Z()
      robotPose = calculateAbsoluteRobotPose(fieldLayout=fieldLayout,tagPose=pose,tagId=16)
      robotX, robotY, robotZ, robotRotation = robotPose.X(),robotPose.Y(),robotPose.Z(),robotPose.rotation().Z()

      robotPoseArray = np.array([robotX,robotZ,robotRotation])

      xTopic.set(x)
      zTopic.set(z)
      rotTopic.set(rotation)
      absolutePoseTopic.set(robotPoseArray)

      print(f"{x}, {y}, {rotation}")

      #piTable.putValue("rotation", rotation)
   # Release the camera and close OpenCV windows
   cap.release()
   cv2.destroyAllWindows()

if __name__ == "__main__":
   main()
