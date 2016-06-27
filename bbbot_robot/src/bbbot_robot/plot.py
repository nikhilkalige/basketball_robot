from ConfigParser import ConfigParser
import zmq
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from enum import IntEnum
from bbbot_robot.dropdown import create_toolbar, create_multi_selector, \
    get_dropdown_value, get_multiselector_value
import time
from pandas import DataFrame
import pickle
import os


class PIdx(IntEnum):
    D_ELBOW = 0
    D_WRIST = 1
    STR_PAN = 2
    STR_LFT = 3
    END_LFT = 4
    STR_ELB = 5
    END_ELB = 6
    STR_WRI = 7
    END_WRI = 8
    DMP_STR = 9


def chunks(l, n):
    n = max(1, n)
    return [l[i:i + n] for i in range(0, len(l), n)]


class Plotter(object):
    BUTTONS = ["elbow_delay", "wrist_delay",
               "angles", "lift_dmp", "elb_dmp", "wri_dmp"]
    MULTI_BUTTONS = ["start_pan", "start_lift", "end_lift",
                     "start_elb", "end_elb", "start_wri", "end_wri"]
    TIME_COLORS = [sns.color_palette()[0], sns.color_palette()[1]]
    SCATTER_COLOR = "#38F538"
    LINE_COLOR = sns.color_palette("bright")[3]
    TIME_NAMES = ["ELBOW TIME", "WRIST TIME"]
    POP_SIZE = 15

    def __init__(self, dump_location, pickle_file=""):
        self.dump_location = dump_location
        self.init_data_structure()
        self.cfg = ConfigParser()
        self.cfg.read("./config/config.cfg")
        self.current_plot = ""
        self.button_value = "elbow_delay"
        self.button_value = "angles"
        self.multi_value = ["", ""]
        if pickle_file:
            with open(pickle_file, 'r') as f:
                self.data = pickle.load(f)
            self.data_updated = True
        else:
            self.data_updated = False

    @property
    def plot_changed(self):
        return self.current_plot != self.button_value

    def init_zmq(self):
        port = self.cfg.get('zmq', 'port')
        context = zmq.Context()
        self.socket = context.socket(zmq.PAIR)
        self.socket.bind("tcp://localhost:%s" % port)

    def init_data_structure(self):
        self.data = dict(
            elbow_delay=[],
            wrist_delay=[],
            start_pan=[],
            start_lift=[],
            end_lift=[],
            start_elb=[],
            end_elb=[],
            start_wri=[],
            end_wri=[],
            dmp=[]
        )

    def recv_msg(self):
        try:
            md = self.socket.recv_json()
            params = self.socket.recv()
            dmps = self.socket.recv()

            params = np.frombuffer(buffer(params), dtype=md['params_dtype'])
            params = np.reshape(params, md['params_shape'])
            dmps = np.frombuffer(buffer(dmps), dtype=md['dmp_dtype'])
            dmps = np.reshape(dmps, md['dmp_shape'])

            self.data["elbow_delay"].append(params[PIdx.D_ELBOW])
            self.data["wrist_delay"].append(params[PIdx.D_WRIST])
            self.data["start_pan"].append(params[PIdx.STR_PAN])
            self.data["start_lift"].append(params[PIdx.STR_LFT])
            self.data["end_lift"].append(params[PIdx.END_LFT])
            self.data["start_elb"].append(params[PIdx.STR_ELB])
            self.data["end_elb"].append(params[PIdx.END_ELB])
            self.data["start_wri"].append(params[PIdx.STR_WRI])
            self.data["end_wri"].append(params[PIdx.END_WRI])
            self.data["dmp"].append(dmps)

            with open(os.path.join(self.dump_location, "data.pkl"), 'wb') as f:
                pickle.dump(self.data, f)
            self.data_updated = True
        except Exception as e:
            print("Mssg read exception: {}".format(e))

    def setup_plot(self, fig=None):
        matplotlib.rcParams['toolbar'] = 'toolmanager'
        sns.set_style("darkgrid", {"font.family": ["fantasy"]})
        self.create_figure()

    def create_figure(self, fig=None):
        self.fig = fig if fig else plt.figure()
        self.fig.subplots_adjust(left=0.05, bottom=0.07, right=0.95, top=0.93)
        create_toolbar(self.fig, self.BUTTONS, self.button_value)
        create_multi_selector(self.fig, self.MULTI_BUTTONS, self.multi_value)

    def time_plot(self):
        if not self.plot_changed:
            if not self.data_updated:
                return
        else:
            plt.close(self.fig)
            self.create_figure()
            plt.clf()
            self.axes = []
            self.axes.append(self.fig.add_subplot(211))
            self.axes.append(self.fig.add_subplot(212))
            self.fig.suptitle(self.button_value.replace("-", " ").capitalize())
            self.axes[0].set_xlabel(
                "Population - {} per generation".format(self.POP_SIZE))
            self.axes[0].set_ylabel("Time (s)")
            self.current_plot = self.button_value

        data = chunks(self.data[self.button_value], self.POP_SIZE)
        start = 0
        for idx in xrange(len(data)):
            self.axes[0].scatter(np.arange(start, start + len(data[idx])), data[idx],
                                 color=self.TIME_COLORS[idx % 2])
            start = start + len(data[idx])

        a = self.axes[0]
        a.set_xticks(np.arange(start, step=self.POP_SIZE))
        self.axes[1] = sns.distplot(
            self.data[self.button_value], ax=self.axes[1], rug=True, bins=10)
        self.data_updated = False

    def dmp_plot(self):
        if not self.plot_changed:
            if not self.data_updated:
                return
        else:
            plt.close(self.fig)
            self.create_figure()
            plt.clf()
            self.axes = []
            self.axes.append(self.fig.add_subplot(211))
            self.axes.append(self.fig.add_subplot(212))
            self.fig.suptitle(self.button_value.replace("-", " ").capitalize())
            self.axes[0].set_xlabel("Time")
            self.axes[0].set_ylabel("Joint Angle (radian)")
            self.current_plot = self.button_value

        idx = ["lift_dmp", "elb_dmp", "wri_dmp"].index(self.button_value)
        data = [arr[idx] for arr in self.data["dmps"]]

        self.axes[0] = sns.tsplot(data, ci=[68, 95], ax=self.axes[0])
        sns.tsplot(data, err_style="unit_traces", ax=self.axes[1])

        # Plot the last population data in different color
        # Get data from the current population
        length = len(data)
        idx = (length // self.POP_SIZE) * self.POP_SIZE
        x_data = np.arange(data[0].shape[0])
        for line in data[idx:]:
            self.axes[1].plot(x_data, line, c=self.LINE_COLOR, ls="--")
        self.data_updated = False

    def angles_plot(self, new=False):
        if not new and not self.plot_changed:
            if not self.data_updated:
                return
        else:
            plt.close(self.fig)
            self.axes = []
            self.current_plot = self.button_value

        data = {key: self.data[key] for key in self.multi_value}
        d_frame = DataFrame(data)
        g = sns.jointplot(x=self.multi_value[0], y=self.multi_value[1], data=d_frame,
                          kind="kde", color="m")
        g.plot_joint(plt.scatter, c="w", s=30, linewidth=1, marker="+")
        g.ax_joint.collections[0].set_alpha(0)
        g.set_axis_labels(self.multi_value[0], self.multi_value[1])

        # Plot the last population data in different color
        # Get data from the current population
        length = len(d_frame)
        idx = (length // self.POP_SIZE) * self.POP_SIZE

        plt.sca(g.ax_joint)
        plt.scatter(x=d_frame[self.multi_value[0]][idx:].tolist(),
                    y=d_frame[self.multi_value[1]][idx:].tolist(),
                    c=self.SCATTER_COLOR)

        self.create_figure(g.fig)
        self.fig.suptitle("Joint Angles")
        self.data_updated = False

    def plot_loop(self):
        plt.ion()
        delay = 1
        while True:
            try:
                self.button_value = get_dropdown_value(self.fig, self.BUTTONS)
                if self.button_value in ["elbow_delay", "wrist_delay"]:
                    self.time_plot()
                if self.button_value in ["angles"]:
                    b1, b2 = get_multiselector_value(
                        self.fig, self.MULTI_BUTTONS)
                    if self.multi_value[0] == b1 and self.multi_value[1] == b2:
                        self.angles_plot(False)
                        self.multi_value = [b1, b2]
                    else:
                        self.multi_value = [b1, b2]
                        self.angles_plot(True)
                if self.button_value in ["lift_dmp", "elb_dmp", "wri_dmp"]:
                    self.dmp_plot()
            except Exception as e:
                print("Exception: ", e)
                # print "exception", self.button_value
            plt.draw()
            plt.pause(0.001)
            time.sleep(delay)


if __name__ == "__main__":
    p = Plotter()
    p.setup_plot()
    p.data["elbow_delay"] = np.random.uniform(0, 0.6, 100).tolist()
    p.data["start_pan"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["start_lift"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["end_lift"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["start_elb"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["end_elb"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["start_wri"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["end_wri"] = np.random.uniform(-3.14, 3.14, 100).tolist()
    p.data["dmps"] = []
    for i in xrange(100):
        p.data["dmps"].append(np.random.uniform(0, 50, (3, 100)))
    p.plot_loop()
