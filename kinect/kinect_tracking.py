
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

class imageProcessor:

    def __init__(self):
        # Create and set logger
        logger = createConsoleLogger(LoggerLevel.Debug)
        setGlobalLogger(logger)

        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        serial = fn.getDeviceSerialNumber(0)
        device = fn.openDevice(serial, pipeline=pipeline)

        listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)

        device.start()

        # NOTE: must be called after device.start()
        registration = Registration(device.getIrCameraParams(),
                                    device.getColorCameraParams())

        undistorted = Frame(512, 424, 4)
        registered = Frame(512, 424, 4)

        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=64,
                        help="max buffer size")
        args = vars(ap.parse_args())

        # define the lower and upper boundaries of the "green" ball
        # in the HSV color space, then initialize the
        # list of tracked points
        greenLower = (40, 45, 86)
        greenUpper = (100, 132, 255)

        pts = deque(maxlen=args["buffer"])

        if os.path.isfile('output.avi'):
            os.remove('output.avi')

        max_val = 0.0


        # allow camera or video file to warm up
        time.sleep(2.0)

        startTime = time.time()
        count_over33 = 0

        # Optional parameters for registration and saving video
        # set True if you need
        need_bigdepth = True
        need_color_depth_map = False
        save_video = False

        bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
        color_depth_map = np.zeros((424, 512),  np.int32).ravel()

        if save_video is True:
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            rgb_video = cv2.VideoWriter("rgb_2.mp4", fourcc, 20, (600, 450))

        frame_dist = time.time()
        while True:

            frames = listener.waitForNewFrame()

            # print time between frames
            start = time.time()
            frame_dist_2 = time.time()
            print("%f.3" % (frame_dist_2 - frame_dist))
            frame_dist = frame_dist_2

            color = frames["color"]
            depth = frames["depth"]
            registration.apply(color, depth, undistorted, registered,
                               bigdepth=bigdepth,
                               color_depth_map=color_depth_map)

            frame = registered.asarray(np.uint8)

            blurred = frame
            # cv2.GaussianBlur(frame, (5, 5), 0)
            image_hsv = time.time()
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            image_pre = time.time()
            image_pre = image_pre - image_hsv

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            cv2.imshow("mask", mask)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            masking = time.time() - image_pre

            # print frame.shape
            # find the contours in the mask and initialize the current
            # (x,y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]

            center = None
            contouring = time.time() - masking

            # only proceed if (cnts > 0)
            radius = 0
            x = 0
            y = 0
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
                X, Y, Z = registration.getPointXYZ(undistorted, int(y), int(x))
                cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
                            (255, 255, 0))

            contour_examine = time.time() - contouring

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

            listener.release(frames)

        device.stop()
        device.close()
        cv2.destroyAllWindows()


x = imageProcessor()

