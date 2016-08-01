#! /usr/bin/env python
from bbbot_robot.config_reader import conf
import sys
import os
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import pickle


sns.set_style("darkgrid", {"font.family": ["fantasy"]})
POP_SIZE = 15
np.set_printoptions(threshold=np.inf)

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

    with open(data_file, 'r') as f:
        data = pickle.load(f)

    # split into blocks of pop size
    length = len(data['fitness'])
    data = [x[0] for x in data['fitness']]

    if length % 15:
        padding = 15 - (length % 15)
        np_data = np.array(data + [0] * padding)
    else:
        np_data = np.array(data)

    blocks = np_data.T.reshape((-1, POP_SIZE))
    # Replace invalid values with 0
    blocks[blocks > 600] = 600
    axes = plt.subplot()

    blocks_list = blocks.tolist()
    # Remove all 600
    stripped_data = []
    for row in blocks_list:
        stripped_data.append([v for v in row if v < 600])

    mean, std = [], []
    for row in stripped_data:
        mean.append(np.mean(row))
        std.append(np.std(row))

    mean = np.array(mean)
    std = np.array(std)

    colors = sns.color_palette('husl')
    x = np.arange(blocks.shape[0])

    # zero stripped data
    stripped_blocks = blocks.copy()
    stripped_blocks[stripped_blocks >= 600] = None
    axes.plot(x, stripped_blocks, "o", color=colors[0], alpha=0.7, markersize=4,
              label="_nolegend_")

    # mean
    axes.plot(x, mean, color=colors[4], linestyle="-")

    # std
    axes.fill_between(x, mean + std, mean - std, color=colors[4], alpha=0.3)
    plt.show()
