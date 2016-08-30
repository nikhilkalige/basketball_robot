from __future__ import print_function
'''
import numpy as np
import imutils
import cv2
import time
import threading
import rospy


class Tracker:
    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)

    greenLower = (27, 100, 50)
    greenUpper = (60, 255, 255)

    def __init__(self):
        connected = False
        self.max_radius = 100000
        self.running = False
        self.killed = False

        while not connected:
            # Bad, but needed
            self.camera = cv2.VideoCapture(1)
            if not self.camera.isOpened():
                rospy.logerr("Unable to connect to the camera, check again")
                time.sleep(5)
            else:
                connected = True

        self.thread = threading.Thread(target=self.video_thread)
        self.thread.start()

    def video_thread(self):
        rospy.loginfo('Camera thread started')

        while not self.killed:
            while self.running:
                (grabbed, frame) = self.camera.read()
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[-2]
                center = None

                if len(cnts) > 0:
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    if radius > 10:
                        cv2.circle(frame, (int(x), int(y)), int(radius),
                                   (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        row = tuple(c[c[:, :, 1].argmin()][0])[1]
                        self.max_radius = min(self.max_radius, row)

                cv2.imshow("Frame", frame)
                cv2.imshow("Masked", mask)
                cv2.waitKey(1) & 0xFF

        cv2.destroyAllWindows()

    def start(self):
        self.max_radius = 100000
        self.running = True

    def stop(self):
        self.running = False

    def kill(self):
        self.killed = True

    def get_reward(self):
        return self.max_radius


if __name__ == '__main__':
    t = Tracker()
    print("Starting thread")
    while True:
        t.start()
        raw_input('Enter to stop: ')
        t.stop()
        print 'Reward', t.get_reward()
        raw_input('Enter to start: ')

'''


import numpy as np
import imutils
import math
import peakutils
import cv2
import time
from peakutils.plot import plot as pplot
import threading
import matplotlib.pyplot as plt
try:
    from pylibfreenect2 import Freenect2, SyncMultiFrameListener
    from pylibfreenect2 import FrameType, Registration, Frame
    from pylibfreenect2 import OpenGLPacketPipeline
    from pylibfreenect2 import CpuPacketPipeline
    from pylibfreenect2 import createConsoleLogger, setGlobalLogger
    from pylibfreenect2 import LoggerLevel

except:
    print("Error importing pylibfreenect")
import time

class Tracker:
    # X_POSITION = -0.11387496
    # Z_POSITION = 1.3
    X_POSITION = -0.185
    Z_POSITION = 1.5


    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)

    greenLower = (27, 100, 50)
    greenUpper = (60, 255, 255)

    def __init__(self, use_kinect=False, basket=False, gui=True):
        self.connected = False
        self.max_radius = 100000
        self.running = False
        self.killed = False
        self.gui = gui
        self.basket = basket
        self.use_kinect = use_kinect
        # self.fig, self.ax = plt.subplots()

        if self.use_kinect:
            self.px = []
            self.py = []
            logger = createConsoleLogger(LoggerLevel.Error)
            setGlobalLogger(logger)

        while not self.connected:
            # Bad, but needed
            if self.use_kinect:
                self.connect_kinect()
            else:
                self.connect_camera()

        self.thread = threading.Thread(target=self.video_thread)
        self.thread.start()

    def connect_camera(self):
        # Bad, but needed
        self.camera = cv2.VideoCapture(1)
        if not self.camera.isOpened():
            print("Unable to connect to the camera, check again")
            time.sleep(5)
        else:
            self.connected = True

    def connect_kinect(self):
        # Bad, but needed
        self.fn = Freenect2()
        num_devices = self.fn.enumerateDevices()
        if num_devices == 0:
            print("Kinect not found, check again")
            self.connected = False
            return

        try:
            self.pipeline = OpenGLPacketPipeline()
        except:
            self.pipeline = CpuPacketPipeline()

        serial = self.fn.getDeviceSerialNumber(0)
        self.device = self.fn.openDevice(serial, pipeline=self.pipeline)

        self.listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        self.device.setColorFrameListener(self.listener)
        self.device.setIrAndDepthFrameListener(self.listener)

        self.device.start()
        print("started")

        # NOTE: must be called after device.start()
        self.registration = Registration(self.device.getIrCameraParams(),
                                         self.device.getColorCameraParams())
        print("registration")
        self.undistorted = Frame(512, 424, 4)
        self.registered = Frame(512, 424, 4)
        self.connected = True

    def video_thread(self):
        print('Camera thread started')
        need_bigdepth = False
        need_color_depth_map = False

        bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
        color_depth_map = np.zeros((424, 512), np.int32).ravel() \
            if need_color_depth_map else None

        while not self.killed:
            while self.running:
                # (grabbed, frame) = self.camera.read()
                frames = self.listener.waitForNewFrame()
                frame = frames["color"]
                ir = frames["ir"]
                depth = frames["depth"]
                self.registration.apply(frame, depth, self.undistorted, self.registered,
                                        bigdepth=bigdepth,
                                        color_depth_map=color_depth_map)

                # print(frame.width, frame.height, frame.bytes_per_pixel)
                # print(ir.width, ir.height, ir.bytes_per_pixel)
                # print(depth.width, depth.height, depth.bytes_per_pixel)
                # print(frame.asarray().shape, ir.asarray().shape, depth.asarray().shape)

                # color_image_array = frame.asarray()
                color_image_array = self.registered.asarray(np.uint8)
                hsv = cv2.cvtColor(color_image_array, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[-2]
                center = None

                if len(cnts) > 0:
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]),
                              int(M["m01"] / M["m00"]))

                    if radius > 10:
                        cv2.circle(color_image_array, (int(x), int(y)), int(radius),
                                   (0, 255, 255), 2)
                        cv2.circle(color_image_array, center, 5, (0, 0, 255), -1)
                        x, y, z = self.registration.getPointXYZ(self.undistorted, y, x)
                        # print(x, y, z, end='\r')
                        if not math.isnan(y) and not math.isnan(z):
                            self.px.append(y)
                            self.py.append(z)
                        row = tuple(c[c[:, :, 1].argmin()][0])[1]
                        self.max_radius = min(self.max_radius, row)

                if self.gui:
                    cv2.imshow("Frame", color_image_array)
                    cv2.imshow("Masked", mask)
                self.listener.release(frames)
                cv2.waitKey(1) & 0xFF
                # self.ax.plot(self.px)
                # self.ax.plot(self.py)
                # plt.pause(0.01)

        cv2.destroyAllWindows()
        print("exit looping")

    def start(self):
        self.max_radius = 100000
        if self.use_kinect:
            self.px = []
            self.py = []
        self.running = True

    def stop(self):
        self.running = False

    def kill(self):
        self.killed = True
        if self.use_kinect:
            self.device.stop()
            self.device.close()

    def get_reward(self):
        if self.use_kinect:
            if not self.basket:
                try:
                    peaks = peakutils.indexes(self.px)
                except Exception as e:
                    print(e)
                    peaks = np.array([])

                if peaks.any():
                    # for xx in peaks:
                    #     t.ax.plot(xx, self.px[xx], 'ro')
                    return self.py[peaks[0]] * 100
                else:
                    return 0
            else:
                print(np.mean(self.px), np.mean(self.py))
                dist = distance(np.array([self.px, self.py]).T, self.X_POSITION, self.Z_POSITION)
                print("distance", dist, len(self.px))
                return dist * 100
        else:
            return self.max_radius


def distance(array, x, z):
    abs_point = np.array([x, z])
    dist = np.linalg.norm(array - abs_point, axis=1)
    return dist.min()


if __name__ == '__main__':
    t = Tracker(use_kinect=True, basket=True)
    print("Starting thread")
    while True:
        t.start()
        raw_input('Enter to stop: ')
        t.stop()
        #V t.ax.cla()
        # t.ax.plot(t.px)
        # t.ax.plot(t.py)
        x = t.get_reward()
        print("reward", x)
        plt.pause(0.01)
        # print 'Reward', t.get_reward()
        raw_input('Enter to start: ')

