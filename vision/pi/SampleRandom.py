import ntcore
from cscore import CameraServer
import numpy as np
import cv2

def main():
    # Initialize ntcore
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClientTeam(1537)

    # Connect to the Limelight network table
    limelight_table = inst.getTable('limelight')

    # Set Limelight pipeline to use
    limelight_table.putNumber('pipeline', 0)

    # Set Limelight LEDs to off
    limelight_table.putNumber('ledMode', 1)

    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Processed Video", 320, 240)

    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    # Initialize HOG descriptor for people detection
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        # Flip the image horizontally for correct orientation
        img = cv2.flip(img, 1)

        # Perform people detection using HOG descriptor
        found, _ = hog.detectMultiScale(img)

        # Draw bounding boxes around detected people
        for (x, y, w, h) in found:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # (optional) send some image back to the dashboard
        outputStream.putFrame(img)