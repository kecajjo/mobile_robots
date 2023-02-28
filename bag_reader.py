#!/usr/bin/env python

from rosbag import Bag

BAG_NAME = "data/2021-12-21-10-57-39.bag"
TOPIC_PREFIX = "/PIONIER6"


class BagReader(object):
    def __init__(self):
        self._bag = Bag(BAG_NAME)

    def get_messages(self, topic_name):
        msgs = []
        for (topic, msg, ts) in self._bag.read_messages(topics=TOPIC_PREFIX+topic_name):
            msgs.append(msg)
        return msgs


if __name__ == '__main__':
    bag_reader = BagReader()
    bag_reader.get_messages("/scan")
