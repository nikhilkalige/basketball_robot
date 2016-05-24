import numpy as np
import imutils
import cv2
import time
import threading


class Tracker:
    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)

    greenLower = (27, 100, 50)
    greenUpper = (60, 255, 255)

    def __init__(self):
        connected = False
        self.max_radius = 100000
        self.running = False

        while not connected:
            # Bad, but needed
            self.camera = cv2.VideoCapture(1)
            if not self.camera.isOpened():
                print "Unable to connect to the camera, check again"
                time.sleep(5)
            else:
                connected = True

        self.thread = threading.Thread(target=self.video_thread)
        self.thread.start()

    def video_thread(self):
        print 'Thread started'

        while True:
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
