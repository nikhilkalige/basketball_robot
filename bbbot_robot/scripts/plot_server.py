from bbbot_robot.plot import Plotter
from bbbot_robot.cmaes import generate_name


plot_dump = "/home/lonewolf/workspace/asu/thesis/cmaes_dump"


if __name__ == '__main__':
    location = generate_name(plot_dump)
    p = Plotter(location)
    p.setup_plot()
    p.init_zmq()
    p.plot_loop()
