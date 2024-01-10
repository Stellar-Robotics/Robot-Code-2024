import cv2

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