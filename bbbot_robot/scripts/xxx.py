import zmq
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import time
import pickle


class Pl(object):
    TIME_COLORS = [sns.color_palette()[0], sns.color_palette()[1]]
    SCATTER_COLOR = "#38F538"
    LINE_COLOR = sns.color_palette("bright")[3]
    TIME_NAMES = ["ELBOW TIME", "WRIST TIME"]
    POP_SIZE = 15
    FITNESS_COLOR = "#FF3A3A"

    def __init__(self):
        self.init_data_structure()
        self.data_updated = False

    def init_data_structure(self):
        self.data = []

    def setup_plot(self, fig=None):
        matplotlib.rcParams['toolbar'] = 'toolmanager'
        sns.set_style("darkgrid", {"font.family": ["fantasy"]})
        self.create_figure()

    def create_figure(self, fig=None):
        self.fig = fig if fig else plt.figure()
        self.fig.subplots_adjust(left=0.05, bottom=0.07, right=0.95, top=0.93)

    def recv_msg(self):
        try:
            md = self.socket.recv_json(flags=zmq.NOBLOCK)
        except Exception as e:
            return
        try:
            params = self.socket.recv()

            params = np.frombuffer(buffer(params), dtype=md['params_dtype'])
            params = np.reshape(params, md['params_shape'])

            self.data.append([params, md['fitness']])
            self.data_updated = True
        except Exception as e:
            print("Mssg read exception: {}".format(e))

        with open('/tmp/groups.pkl', 'wb') as f:
            pickle.dump(self.data, f)

    def init_zmq(self):
        port = 45669
        context = zmq.Context()
        self.socket = context.socket(zmq.PAIR)
        self.socket.bind("tcp://127.0.0.1:%s" % port)

    def data_plot(self):
        if self.data_updated:
            data = [item[1] for item in self.data]
            plt.plot(data, 'ro')
            plt.ylim(0, 200)
            self.data_updated = False

    def plot_loop(self):
        plt.ion()
        delay = 0.5
        while True:
            try:
                self.data_plot()
            except Exception as e:
                print("Exception: ", e)
            plt.draw()
            plt.pause(0.001)
            time.sleep(delay)
            self.recv_msg()


if __name__ == '__main__':
    print("hiiii")
    p = Pl()
    p.init_zmq()
    p.plot_loop()
