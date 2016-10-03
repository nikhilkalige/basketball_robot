#!/usr/bin/env python
from bbbot_robot.plot import Plotter
from bbbot_robot.cmaes import generate_name
from bbbot_robot.config_reader import conf


if __name__ == '__main__':
    location = generate_name(conf.get('dump', 'plotter_dump'))

    pfile = "/home/nikhilkalige/workspace/asu/thesis/plotter_dump/" + "23_08_18_54_28" + '/data.pkl'
    p = Plotter(location, pickle_file=pfile)

    #p = Plotter(location)

    p.setup_plot()
    p.init_zmq()
    p.plot_loop()
