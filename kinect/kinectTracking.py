
import imutils
import numpy as np
import cv2
import time
import sys
import os
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
from  tracker import TD2ndTracker
import matplotlib.pyplot as plt
import math
import serial
import struct


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
    #theta = -601*math.atan((x-0.204)/(z+0.265)) -48.4
    base = math.atan((x - 0.315) / (z + 0.265))
    theta = -652 * base + 43.2
    phi = 574 * math.atan((y + 0.34) / (z + 0.18 - 0.055 * math.cos(theta * math.pi / 180))) - 35.4
    return [theta, phi]



class ImageProcessor:

    def __init__(self):
        dt = 0.066 # time step
        R_std = 0.0
        Q_std = 0.1
        self.tracker = TD2ndTracker(Q_std, R_std, dt)
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.prevFrame = time.time()
        self.xs, self.ys, self.zrs = [], [], []
        self.xrs, self.yrs, self.zrrs = [], [], []
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
        greenLower = (36, 90, 40)
        greenUpper = (255, 255, 255)

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
                get_angles(X, Y, Z)
                self.tracker.predict()
                x = [X, Y, Z]
                self.tracker.update(x)
                cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
                            (255, 255, 0))
                self.xs.append(self.tracker.tracker.x[0])
                self.ys.append(self.tracker.tracker.x[1])
                self.zrs.append(self.tracker.tracker.x[2])
                self.xrs.append(X)
                self.yrs.append(Y)
                self.zrrs.append(Z)
            else:
                self.tracker.predict()
                x = [self.tracker.tracker.x[0], self.tracker.tracker.x[1], self.tracker.tracker.x[2]]
                self.tracker.update(x)
                self.xs.append(self.tracker.tracker.x[0])
                self.ys.append(self.tracker.tracker.x[1])
                self.zrs.append(self.tracker.tracker.x[2])

            if len(self.xs) > 200:
                print("wheres my plot?")
                #plt.close()
                plt.plot(self.xs, self.ys, 'ro')
                plt.plot(self.xrs, self.yrs, 'bo')
                plt.show()
                print(self.xs)
                self.xs = []
                self.ys = []
                self.zrs = []
            end = time.time()
            if end - start > max_val and end - startTime > 1:
                max_val = end - start

            if end - start > 0.066:
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
        greenLower = (36, 90, 40)
        greenUpper = (255, 255, 255)

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
            cv2.putText(frame, 'X = %.3f Y = %.3f Z = %.3f' % (self.X, self.Y, self.Z), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1,
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
print(os.getcwd())
time.sleep(2.0)
Fire = 255
target = False
prev_angles = [0, 0]
# imageRunner.runContinuous()
velocity = []
Real = []
pred5 = []
pred10 = []
pred15 = []
pred30 = []
prev_X = 0
prev_Y = 0
prev_Z = 0
prev_Time = 0
beenThrown = False
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=3)
#time.sleep(22)
while True:
    X, Y, Z, fTime = imageRunner.runSingleFrame()
    velocity.append([(X-prev_X)/(fTime-prev_Time),(Y-prev_Y)/(fTime-prev_Time),(Z-prev_Z)/(fTime-prev_Time)])
    prev_X = X
    prev_Y = Y
    prev_Z = Z
    Real.append([X,Y,Z])

    if velocity[-1][1] < -4:
        beenThrown = True
    elif velocity[-1][1] > 2:
        beenThrown = False
    if len(velocity) >= 5 and beenThrown is False:
        X_avg = (velocity[-1][0] + velocity[-2][0] + velocity[-3][0] + velocity[-4][0] + velocity[-5][0])/5
        Y_avg = (velocity[-1][1] + velocity[-2][1] + velocity[-3][1] + velocity[-4][1] + velocity[-5][1]) / 5
        Z_avg = (velocity[-1][2] + velocity[-2][2] + velocity[-3][2] + velocity[-4][2] + velocity[-5][2]) / 5
        pred5.append([X + X_avg*5*0.033,Y + Y_avg*5*0.033,Z+Z_avg*5*0.033])
        pred10.append([X + X_avg*10*0.033,Y + Y_avg*10*0.033,Z+Z_avg*10*0.033])
        pred15.append([X + X_avg * 15*0.033, Y + Y_avg * 15*0.033, Z + Z_avg * 15*0.033])
        pred30.append([X + X_avg * 30*0.033, Y + Y_avg * 30*0.033, Z + Z_avg * 30*0.033])
    elif len(velocity) >= 5:
        X_avg = (velocity[-1][0] + velocity[-2][0] + velocity[-3][0] + velocity[-4][0] + velocity[-5][0]) / 5
        Y_avg = (velocity[-1][1] + velocity[-2][1] + 9.81*0.033)/2
        Z_avg = (velocity[-1][2] + velocity[-2][2] + velocity[-3][2] + velocity[-4][2] + velocity[-5][2]) / 5
        pred5.append([X + X_avg * 5 * 0.0333, Y + Y_avg * 5 * 0.0333 + 1/2*9.81*(5*0.0333)**2, Z + Z_avg * 5 * 0.0333])
        pred10.append([X + X_avg * 10 * 0.0333, Y + Y_avg * 10 * 0.0333 + 1/2*9.81*(10*0.0333)**2, Z + Z_avg * 10 * 0.0333])
        pred15.append([X + X_avg * 15 * 0.0333, Y + Y_avg * 15 * 0.0333 + 1/2*9.81*(15*0.0333)**2, Z + Z_avg * 15 * 0.0333])
        pred30.append([X + X_avg * 30 * 0.0333, Y + Y_avg * 30 * 0.0333 + 1/2*9.81*(30*0.0333)**2, Z + Z_avg * 30 * 0.0333])
    print(velocity[-1][0], velocity[-1][1], velocity[-1][2], fTime - prev_Time)
    prev_Time = fTime
    #print("%f, %f, %f, %d" % (X, Y, Z, fTime))
    angles = get_angles(X, Y, Z)
    # print('X = %.3f Y = %.3f Z = %.3f' % (X, Y, Z))
    # print('phi = %.3f, theta = %.3f' % (angles[0], angles[1]))
    angleDelta = max([abs(prev_angles[0]-angles[0]),abs(prev_angles[1]-angles[1])])
    if angles[0] > -400 and angles[0] < 400 and angles[1] > -600 and angles[1] < 600 and angleDelta > 10:
        prev_angles = [angles[0], angles[1]]
        ser.write(struct.pack('>bhhhb', 36, Fire, int(angles[0]), int(angles[1]), 47))

    key = cv2.waitKey(1) & 0xFF

    # if 'q' is pressed stop the loop
    if key == ord("q"):
        break
    elif key == ord(" "):
        Fire = 0
Realfile = open('values.csv','w+')
for i in range(len(Real)):
    Realfile.write('%f,%f,%f' % (Real[i][0],Real[i][1],Real[i][2]))
    if i >= 9:
        Realfile.write(',%f,%f,%f' % (pred5[i-5][0],pred5[i-5][1],pred5[i-5][2]))
    if i >= 19:
        Realfile.write(',%f,%f,%f' % (pred15[i-15][0],pred15[i-15][1],pred15[i-15][2]))
    if i >= 34:
        Realfile.write(',%f,%f,%f' % (pred30[i-30][0],pred30[i-30][1],pred30[i-30][2]))
        Realfile.write('%f,%f,%f' % (velocity[i][0], velocity[i][1], velocity[i][2]))
    Realfile.write('\n')
Realfile.close()
imageRunner.closeKinect()
ser.close()



