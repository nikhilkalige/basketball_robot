import threading
import rospy
from gazebo_msgs.msg import ModelStates


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
