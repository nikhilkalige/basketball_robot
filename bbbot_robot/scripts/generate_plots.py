#!/usr/bin/env python
from bbbot_robot.plot import Plotter
from bbbot_robot.config_reader import conf
import sys
import os
import tempfile


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Enter folder location")
        sys.exit()

    folder_name = sys.argv[1]
    folder = conf.get('dump', 'plotter_dump')
    loc = os.path.join(folder, folder_name)

    if not os.path.exists(loc):
        print("Invalid path {}".format(loc))
        sys.exit()

    data_file = os.path.join(loc, 'data.pkl')
    if not os.path.exists(data_file):
        print('data.pkl doesn\'t exist in {}'.format(loc))
        sys.exit()

    location = '/tmp/'
    output_location = tempfile.mkdtemp()
    p = Plotter(location, pickle_file=data_file)
    p.setup_plot()
    p.save_plots(output_location)
    print("Output generated at {}".format(output_location))
