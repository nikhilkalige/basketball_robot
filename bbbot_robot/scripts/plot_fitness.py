#! /usr/bin/env python
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

    if not os.path.exists(sys.argv[1]):
        print("Invalid path {}".format(sys.argv[1]))
        sys.exit()

    loc = sys.argv[1]
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

    mean = blocks.mean(axis=1)
    std = blocks.std(axis=1)

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
