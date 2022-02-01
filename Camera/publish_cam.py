#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def talker():
    pub = rospy.Publisher('cool_topic', Image, queue_size=10)
    rospy.init_node('best_node_ever', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish('/camera/color/camera_info')
        rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
