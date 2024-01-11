import cv2
import os

inputType = "img"

if inputType == "img":
   # Must be in project root for file path to register correctly
   imcap = cv2.imread(os.getcwd() + "/vision/sample_image.png", cv2.IMREAD_COLOR)

   while(True):
      cv2.imshow("PlzWork", imcap)
      if cv2.waitKey(1) & 0xFF==ord('q'):
         break

if inputType == "vid":

   vidcap = cv2.VideoCapture(0)

   if vidcap.isOpened():
      ret, frame = vidcap.read()
      if ret:
         while(True):
               cv2.imshow("PlzWork",frame)
               if cv2.waitKey(1) & 0xFF==ord('q'):
                  break
      else:
         print("frame not captured")
   else:
      print("cannot open camera")