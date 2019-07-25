#!/usr/bin/env python2
import sys

import rosbag


def main():
    bag = rosbag.Bag(sys.argv[1])
    topics = bag.get_type_and_topic_info()[1].keys()
    data = {}
    for topic in topics:
        data[topic] = [msg for _, msg, t in bag.read_messages(topic)]


if __name__ == '__main__':
    main()
