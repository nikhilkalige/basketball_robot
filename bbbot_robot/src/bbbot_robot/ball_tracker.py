import threading
import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np


class BallTracker:
    BALL_RADIUS = 0.13

    def __init__(self):
        self.distance = 0
        self.z = 10
        self.running = False
        self.killed = False
        self.first_read = False

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_cb)
        self.thread = threading.Thread(target=self.callback_thread)
        self.thread.start()

    def callback_thread(self):
        rospy.loginfo('Ball tracking thread started')
        rospy.spin()

    def model_cb(self, data):
        if not self.running:
            return
        try:
            idx = data.name.index("basketball")
        except ValueError:
            return
        pos = data.pose[idx]
        if self.first_read:
            self.first_read = False
            if pos.position.z <= (self.BALL_RADIUS + 0.002):
                # This means the ball is already on the ground, this should return invalid value
                self.z = 0
                self.distance = 0

        if pos.position.z < self.z and pos.position.z > 0:
            self.distance = pos.position.x
            self.z = pos.position.z

    def start(self):
        self.distance = 0
        self.z = 10
        self.first_read = True
        self.running = True

    def stop(self):
        self.running = False

    def kill(self):
        pass

    def get_reward(self):
        print (self.distance * 100) if self.distance > 0.4 else 0
        return (self.distance * 100) if self.distance > 0.4 else 0


class BasketTracker(BallTracker):
    X_POSITION = 2.5
    Z_POSITION = 1.6
    Y_POSITION = 0.0
    Z_HOOP_HEIGHT = 1.2

    def model_cb(self, data):
        if not self.running:
            return
        try:
            idx = data.name.index("basketball")
        except ValueError:
            return
        pos = data.pose[idx]
        # print(pos.position.x, pos.position.z)
        self.position.append([pos.position.x, pos.position.y, pos.position.z])

    def start(self):
        self.position = []
        self.distance = 0
        self.z = 10
        self.first_read = True
        self.running = True

    def get_reward(self):
        positions = np.array(self.position)

        # Check if there is a dunk
        if (positions[:, 2].min() >= 1.2) and (positions[:, 0].max() >= 1.5):
            print("Robot has dunked the ball")
            return 0

        # See if ball already on the ground
        if positions[0][2] < 0.65:
            return 800

        # dist = distance(positions, self.X_POSITION, self.Z_POSITION)
        dist = distanceY(positions, self.X_POSITION, self.Y_POSITION, self.Z_POSITION)
        # The ball on the ground usually returns a value of 1.47, so any ball that falls
        # very close to the robot and then rolls over gets the same reward, therefore we
        # use find the point where the ball hits the ground and then calculate the distance
        # from there.
        if dist > 1.46:
            z = positions[:, 2]
            zd = np.diff(z)
            stop_idx = zd.argmin()
            xm, ym, zm = positions[stop_idx]
            sdist = np.linalg.norm(positions[stop_idx] - np.array([self.X_POSITION, self.Y_POSITION, self.Z_POSITION]))
            dist = max(dist, sdist)

        print(dist * 100)
        return dist * 100


def distance(array, x, z):
    abs_point = np.array([x, z])
    dist = np.linalg.norm(array - abs_point, axis=1)
    return dist.min()


def distanceY(array, x, y, z):
    abs_point = np.array([x, y, z])
    dist = np.linalg.norm(array - abs_point, axis=1)
    return dist.min()


if __name__ == '__main__':
    rospy.init_node("ball_tracker")
    t = BallTracker()
    print("Starting thread")
    while True:
        t.start()
        raw_input('Enter to stop: ')
        t.stop()
        print 'Reward', t.get_reward()
        raw_input('Enter to start: ')
