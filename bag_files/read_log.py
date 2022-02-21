#!/usr/bin/env python

import rosbag

if __name__=="__main__":
    bag = rosbag.Bag('test.bag')
    for (topic, msg, t) in bag.read_messages():
        print(topic, msg, t)
