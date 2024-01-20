from cscore import CameraServer
import ntcore

import cv2
import json
import numpy as np
import os
import time
import robotpy_apriltag as apriltag


def main():
   with open('/boot/frc.json') as f:
      config = json.load(f)
   camera = config['cameras'][0]

   width = camera['width']
   height = camera['height']

   nt = ntcore.NetworkTableInstance.getDefault()

   CameraServer.startAutomaticCapture()

   input_stream = CameraServer.getVideo()
   output_stream = CameraServer.putVideo('Processed', width, height)

   # Table for vision output information
   vision_nt = nt.getTable('Vision')

   detector = apriltag.AprilTagDetector()
   detector.addFamily("tag36h11")
    #config = apriltag.AprilTagPoseEstimator.Config(tagSize, focalLengthX, focalLengthY, focalCenterX, focalCenterY)
    #estimator = apriltag.AprilTagPoseEstimator(config)
   prevId = False

   # Allocating new images is very expensive, always try to preallocate
   img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

   # Wait for NetworkTables to start
   time.sleep(0.5)

   while True:
      start_time = time.time()

      frame_time, input_img = input_stream.grabFrame(img)
      output_img = np.copy(input_img)

      # Notify output of error and skip iteration
      if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue
      
      gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)

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
                os.system('clear')
                continue
            else:
                os.system('cls' if os.name=='nt' else 'clear') # clears the console window
                #os.system(f"notify-send \"April tag detected: ID = {tag_id}\"" if os.name!='nt' else '') # Sends unix graphical notification if possible
                print(f"Tag ID: {tag_id}") # Displays the tag_id in the console

      # Convert to HSV and threshold image
      '''hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
      binary_img = cv2.inRange(hsv_img, (0, 0, 100), (85, 255, 255))

      _, contour_list = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

      x_list = []
      y_list = []

      for contour in contour_list:

            # Ignore small contours that could be because of noise/bad thresholding
            if cv2.contourArea(contour) < 15:
               continue

            cv2.drawContours(output_img, contour, -1, color=(255, 255, 255), thickness=-1)

            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            center = tuple([int(dim) for dim in center])  # Convert to int so we can draw

            # Draw rectangle and circle
            cv2.drawContours(output_img, [cv2.boxPoints(rect).astype(int)], -1, color=(0, 0, 255), thickness=2)
            cv2.circle(output_img, center=center, radius=3, color=(0, 0, 255), thickness=-1)

            x_list.append((center[0] - width / 2) / (width / 2))
            x_list.append((center[1] - width / 2) / (width / 2))

      vision_nt.putNumberArray('target_x', x_list)
      vision_nt.putNumberArray('target_y', y_list)

      processing_time = time.time() - start_time
      fps = 1 / processing_time
      cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))'''
      output_stream.putFrame(output_img)


main()