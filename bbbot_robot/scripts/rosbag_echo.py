import rosbag
import sys


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit()

    count = 0
    with rosbag.Bag(sys.argv[1], 'r') as bag:
        for _, msg, _ in bag.read_messages():
            print count, ":", ["{:.7f}".format(i) for i in msg.position]
            count += 1
