import cv2
import numpy as np
import robotpy_apriltag as apriltag
import ntcore
from wpimath.geometry import *
from wpimath.filter import LinearFilter
import time

def calculateAbsoluteRobotPose(fieldLayout : apriltag.AprilTagFieldLayout, tagPose : Transform3d, tagId : int):
    # closest thing to assumed odometry coordinate system is EDN, so we'll convert the absolute pose to that
    absoluteTagPose = CoordinateSystem.convert(fieldLayout.getTagPose(tagId),CoordinateSystem.NWU(),CoordinateSystem.EDN())
    return absoluteTagPose.transformBy(tagPose.inverse())
    
    #absRobotW = absoluteTagPose.Y() - tagPose.Y()
    #absRobotN = absoluteTagPose.X() - tagPose.X()
    #absRobotU = absoluteTagPose.Z() - tagPose.Z()

def main():
   tagSize = 0.17 # meters
   focalCenterX = 325.00253538
   focalCenterY = 235.65891798
   focalLengthX = 656.29804936
   focalLengthY = 655.66760244

   inst = ntcore.NetworkTableInstance.getDefault()
   inst.setServerTeam(5413)
   inst.startClient4("raspberrypi")

   smartDashboard = inst.getTable("SmartDashboard")
   xTopic = smartDashboard.getDoubleTopic("x").publish()
   zTopic = smartDashboard.getDoubleTopic("z").publish()
   rotTopic = smartDashboard.getDoubleTopic("rot").publish()
   frameXTopic = smartDashboard.getDoubleTopic("frameX").publish()
   frameZTopic = smartDashboard.getDoubleTopic("frameZ").publish()

   absolutePoseTopic = smartDashboard.getDoubleArrayTopic("robotPose").publish()

   tagsToFollow = [4, 7, 11, 12, 13, 14, 15, 16]

   frameZFilter = LinearFilter.singlePoleIIR(0.5, 0.2)
   
   # Initialize the camera
   cap = cv2.VideoCapture(0)

   # Create an AprilTag detector
   detector = apriltag.AprilTagDetector()
   detector.addFamily("tag36h11")
   config = apriltag.AprilTagPoseEstimator.Config(tagSize, focalLengthX, focalLengthY, focalCenterX, focalCenterY)
   estimator = apriltag.AprilTagPoseEstimator(config)
   #field = apriltag.AprilTagField.k2024Crescendo
   #fieldLayout = apriltag.loadAprilTagLayoutField(field=field)
   fieldLayout = apriltag.AprilTagFieldLayout("fieldLayout.json") # Uncomment if the load method doesn't work


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
      
      
      tagToFollow = next(filter(lambda x: x.getId() in tagsToFollow, tags), None)

      # If the tag wasn't found, skip the rest of the loop
      if tagToFollow is None:
         absolutePoseTopic.set(np.array([]))
         frameXTopic.set(300)
         continue
      
      
      pose = estimator.estimate(tagToFollow)

      x, y, z, rotation = pose.X(), pose.Y(), pose.Z(), pose.rotation().Z()

      robotPose = calculateAbsoluteRobotPose(fieldLayout=fieldLayout,tagPose=pose,tagId=16)
      robotX, robotZ, robotRotation = robotPose.X(),robotPose.Z(),robotPose.rotation().Z()
      # Get the Y from NWU since the normal coordinate system is EDN
      robotY = CoordinateSystem.convert(robotPose, CoordinateSystem.EDN(), CoordinateSystem.NWU()).Y() 

      robotPoseArray = np.array([robotX,robotZ,robotRotation])

      xTopic.set(x)
      zTopic.set(z)
      rotTopic.set(rotation)
      absolutePoseTopic.set(robotPoseArray)

      frameXTopic.set(tagToFollow.getCenter().x)
      frameZTopic.set(pose.z)

      #print(f"{x}, {y}, {z}, {rotation}")
      #print(f"Robot Coordinates (EUN): {robotX}, {robotY}, {robotZ}, {rotation} - Tag pose (EDN?): {x}, {y}, {z}")
      print(frameZFilter.calculate(pose.z))

      #cv2.imshow("frame", frame)

      time.sleep(0.1)
      #cv2.waitKey(100)

      #piTable.putValue("rotation", rotation)
   # Release the camera and close OpenCV windows
   cap.release()
   cv2.destroyAllWindows()

if __name__ == "__main__":
   main()
