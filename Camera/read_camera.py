#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('cool_topic', anonymous=True)
    #rospy.Subscriber('/camera/color/camera_info', CameraInfo, callback)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
