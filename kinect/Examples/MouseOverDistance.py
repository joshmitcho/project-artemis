import imutils
import subprocess
import numpy as np
import cv2
import time
import sys
from collections import deque
import argparse
import os
print(os.getcwd())
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
import matplotlib.pyplot as plt

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


x_st = 0
y_st = 0
def on_click(event, x, y, flags, param):
    x_st = x
    y_st = y

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
window_name="hello"
cv2.namedWindow( window_name, cv2.WINDOW_AUTOSIZE );
cv2.setMouseCallback(window_name, on_click)

while True:

    frames = listener.waitForNewFrame()

    # print time between frames
    start = time.time()
    frame_dist2 = time.time()
    # print("%f.3" % (frame_dist2 - frame_dist))
    frame_dist = frame_dist2

    color = frames["color"]
    depth = frames["depth"]
    registration.apply(color, depth, undistorted, registered,
                            bigdepth=bigdepth,
                            color_depth_map=color_depth_map)

    frame = registered.asarray(np.uint8)

    # show the frame to our screen

    X, Y, Z = registration.getPointXYZ(undistorted, int(y_st), int(x_st))
    cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
                            (255, 255, 0))
    cv2.imshow(window_name, frame)
    key = cv2.waitKey(1) & 0xFF

    # if 'q' is pressed, stop the loop
    if key == ord("q"):
        break

    listener.release(frames)