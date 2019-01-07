
import imutils
import numpy as np
import cv2
import time
import sys
from collections import deque
import argparse
import os
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel



try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)


class ImageProcessor:

    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.prevFrame = time.time()

        self.fn = Freenect2()
        num_devices = self.fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        serial = self.fn.getDeviceSerialNumber(0)
        self.device = self.fn.openDevice(serial, pipeline=pipeline)

        self.listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        self.device.setColorFrameListener(self.listener)
        self.device.setIrAndDepthFrameListener(self.listener)

        self.device.start()
        # NOTE: must be called after device.start()
        self.registration = Registration(self.device.getIrCameraParams(),
                                         self.device.getColorCameraParams())

    def runContinuous(self):

        undistorted = Frame(512, 424, 4)
        registered = Frame(512, 424, 4)

        # define the lower and upper boundaries of the "green" ball
        # in the HSV color space
        greenLower = (40, 45, 86)
        greenUpper = (100, 132, 255)

        max_val = 0.0

        # allow camera or video file to warm up
        time.sleep(2.0)

        startTime = time.time()
        count_over33 = 0

        bigdepth = Frame(1920, 1082, 4)
        color_depth_map = np.zeros((424, 512),  np.int32).ravel()

        frame_dist = time.time()
        while True:

            frames = self.listener.waitForNewFrame()

            # print time between frames
            start = time.time()
            frame_dist2 = time.time()
            # print("%f.3" % (frame_dist2 - frame_dist))
            frame_dist = frame_dist2

            color = frames["color"]
            depth = frames["depth"]
            self.registration.apply(color, depth, undistorted, registered,
                                    bigdepth=bigdepth,
                                    color_depth_map=color_depth_map)

            frame = registered.asarray(np.uint8)

            blurred = frame
            # cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            cv2.imshow("mask", mask)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # print frame.shape
            # find the contours in the mask and initialize the current
            # (x,y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]

            # only proceed if (cnts > 0)
            radius = 0
            if len(cnts) > 0:

                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10.0:
                    # print(radius)

                    # draw the circle and centroid on the frame
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # if radius exists print XYZ from camera
            if int(radius) is not 0:
                X, Y, Z = self.registration.getPointXYZ(undistorted, int(y), int(x))
                cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
                            (255, 255, 0))

            end = time.time()
            if end - start > max_val and end - startTime > 1:
                max_val = end - start

            if end - start > 0.033:
                count_over33 = count_over33 + 1
            print(str(count_over33))

            # show the frame to our screen
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

            # if 'q' is pressed, stop the loop
            if key == ord("q"):
                break

            self.listener.release(frames)

        self.device.stop()
        self.device.close()
        cv2.destroyAllWindows()

    def runSingleFrame(self):
        undistorted = Frame(512, 424, 4)
        registered = Frame(512, 424, 4)

        # define the lower and upper boundaries of the "green" ball
        # in the HSV color space
        greenLower = (40, 45, 86)
        greenUpper = (100, 132, 255)

        max_val = 0

        bigdepth = Frame(1920, 1082, 4)
        color_depth_map = np.zeros((424, 512), np.int32).ravel()

        frames = self.listener.waitForNewFrame()
        currentFrameArrival = time.time()

        color = frames["color"]
        depth = frames["depth"]
        self.registration.apply(color, depth, undistorted, registered,
                                bigdepth=bigdepth,
                                color_depth_map=color_depth_map)

        frame = registered.asarray(np.uint8)

        blurred = frame
        # cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        cv2.imshow("mask", mask)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # print frame.shape
        # find the contours in the mask and initialize the current
        # (x,y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # only proceed if (cnts > 0)
        radius = 0
        if len(cnts) > 0:

            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10.0:
                # print(radius)

                # draw the circle and centroid on the frame
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # if radius exists print XYZ from camera
        if int(radius) is not 0:
            self.X, self.Y, self.Z = self.registration.getPointXYZ(undistorted, int(y), int(x))
            cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
                        (255, 255, 0))

        # show the frame to our screen
        cv2.imshow("Frame", frame)

        self.listener.release(frames)
        if int(radius) is not 0:
            return self.X, self.Y, self.Z, currentFrameArrival
        else:
            return float('inf'), float('inf'), float('inf'), currentFrameArrival

    def closeKinect(self):
        self.device.stop()
        self.device.close()
        cv2.destroyAllWindows()

        
imageRunner = ImageProcessor()
time.sleep(2.0)
while True:
    X, Y, Z, fTime = imageRunner.runSingleFrame()
    print('X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z))
    key = cv2.waitKey(1) & 0xFF

    # if 'q' is pressed, stop the loop
    if key == ord("q"):
        break

imageRunner.closeKinect()

