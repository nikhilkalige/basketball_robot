#! /usr/bin/env python
from bbbot_robot.evaluate import EvaluateGazebo
from bbbot_robot.config_reader import conf
from pykeyboard import PyKeyboard
import sys
import os
import rospy
try:
    from PySide import QtCore
    from PySide import QtGui
    from PySide.QtCore import Slot, QThread, Signal, QObject, QCoreApplication
except ImportError as e:
    print(e)
    from PyQt4 import QtCore
    from PyQt4 import QtGui
    from PyQt4.QtCore import QThread, Signal


DMP = 15
BAG_FILE = '/home/nikhilkalige/workspace/ros/devel_packages/bag_files/2016-05-15-15-20-40.bag'


def trigger_kazam(key_value):
    keyboard = PyKeyboard()
    keyboard.press_key(keyboard.super_l_key)
    keyboard.press_key(keyboard.control_key)
    keyboard.tap_key(key_value)
    keyboard.release_key(keyboard.super_l_key)
    keyboard.release_key(keyboard.control_key)


def kazam_pause():
    trigger_kazam('p')


def kazam_stop():
    trigger_kazam('f')


class Event(QObject):
    data = Signal(float, float)


class robot_thread(QThread):

    def __init__(self, location, video):
        QThread.__init__(self)
        self.loc = location
        self.video = video
        self.event = Event()

    def run(self):
        robot = EvaluateGazebo(DMP, BAG_FILE, plot=False)
        if video:
            robot.register_run_callback(kazam_pause, kazam_pause)

        rospy.loginfo("Starting replay")

        with open(data_file, 'r') as f:
            num_lines = sum(1 for line in f) - 1
            print("Iterations count: {}".format(num_lines))

        with open(data_file, 'r') as f:
            f.readline()  # skip first line
            idx = 1
            for line in f.readlines():
                print("Evalution: {}/{}".format(idx, num_lines))
                params = [float(x) for x in line.split()[5:]]
                self.event.data.emit(idx, " ")
                robot.eval(params)
                idx += 1

        if self.video:
            kazam_stop()
        robot.track.kill()
        rospy.loginfo("Finished execution")
        QCoreApplication.quit()


class Demo(QtGui.QWidget):
    def __init__(self, location, video):
        super(Demo, self).__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }")
        self.setStyleSheet("QWidget { font-size:18px; background-color : rgba(100, 100, 100, 255); color : white; }")

        x, y, w, h = 1600, 100, 220, 50
        self.setGeometry(x, y, w, h)
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)

        label1 = QtGui.QLabel("Generation: ", self)
        x, y = 5, 5
        label1.move(x, y)

        self.generation = QtGui.QLabel(" " * 15, self)
        x, y = 115, 5
        self.generation.move(x, y)

        label2 = QtGui.QLabel("Reward: ", self)
        x, y = 5, 25
        label2.move(x, y)

        self.reward = QtGui.QLabel(" " * 15 + '0 cm', self)
        x, y = 115, 25
        self.reward.move(x, y)

        self.robot_thread = robot_thread(location, video)

    def show_and_raise(self):
        self.robot_thread.event.data.connect(self.set_value)
        self.robot_thread.start()
        self.show()
        self.raise_()

    @Slot(float, float)
    def set_value(self, gen, reward):
        self.generation.setText(str(gen))
        self.reward.setText(str(reward) + ' cm')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Enter folder name")
        sys.exit()

    folder_name = sys.argv[1]
    folder = conf.get('dump', 'cmaes_dump')
    loc = os.path.join(folder, folder_name)

    if not os.path.exists(loc):
        print("Invalid path {}".format(loc))
        sys.exit()

    data_file = os.path.join(loc, 'outcmaesxmean.dat')
    if not os.path.exists(data_file):
        print('outcmaesxmean.dat doesn\'t exist in {}'.format(loc))
        sys.exit()

    video = False
    if len(sys.argv) > 2:
        if '--video' == sys.argv[2]:
            video = True

    rospy.init_node('bot', log_level=rospy.WARN)
    app = QtGui.QApplication(sys.argv)
    demo = Demo(data_file, video)
    demo.show_and_raise()
    sys.exit(app.exec_())
