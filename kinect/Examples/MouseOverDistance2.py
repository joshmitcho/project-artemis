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
import math
import serial
import time
import struct

def get_angles(x, y, z):
    # x_of = 0.204
    # y_of = 0.34
    # z_of = 0.265
    # z_blaster = z + z_of
    # y_blaster = y + y_of
    # x_blaster = x + x_of
    # phi = math.atan(x_blaster / z_blaster) * 360 / 2 / math.pi
    # theta = math.atan(y_blaster/(z_blaster - 0.068*math.cos(phi)))*360 /2 / math.pi

    # try:
    #     phi = 1./math.atan(500.0*(200.*z+53.)/(2.*(500.*x + 69.)))*360./2.
    # except:
    #     phi = 0
    base = math.atan((x-0.315)/(z+0.265))
    theta = -652*base - 38.6
    phi = 574*math.atan((y+0.34)/(z+0.18-0.055*math.cos(theta*math.pi/180))) -35.4
    return [theta, phi]

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


#ser = serial.Serial('/dev/ttyACM0', 9600, timeout=3)
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


refPt = [[0, 0]]
clicked = [0]


def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt.append([y, x])
        clicked.append(clicked[-1] + 1)

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

clickwait = 0
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

    X, Y, Z = registration.getPointXYZ(undistorted, int(refPt[-1][0]), int(refPt[-1][1]))
    angles = get_angles(X, Y, Z)
    cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f ' % (X, Y, Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
                            (255, 0, 0))
    cv2.putText(frame, 'theta = %.3f phi = %.3f ' % (angles[0], angles[1]), (50, 100),
                cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 255, 0))
    # if clicked[-1] > clickwait and angles[0] > -40 and angles[0] < 40 and angles[1] > -60 and angles[1] < 60:
    #     target = True
    #     print('phi = %.3f, theta = %.3f' % (angles[0], angles[1]))
    #     ser.write(struct.pack('>bhhhb', 36, 30, int(angles[1]*-10), int(angles[0]*10), 47))
    #     clickwait += 1
    frame[int(refPt[-1][0]), int(refPt[-1][1])] = (0,0,255,0)
    cv2.imshow(window_name, frame)
    key = cv2.waitKey(1) & 0xFF

    # if 'q' is pressed, stop the loop
    if key == ord("q"):
        break

    listener.release(frames)