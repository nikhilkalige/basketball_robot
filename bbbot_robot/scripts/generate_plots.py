#!/usr/bin/env python
from bbbot_robot.plot import Plotter
import sys
import os
import tempfile


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Enter folder location")
        sys.exit()

    if not os.path.exists(sys.argv[1]):
        print("Invalid path {}".format(sys.argv[1]))
        sys.exit()

    loc = sys.argv[1]
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
