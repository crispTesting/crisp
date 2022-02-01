#!/usr/bin/env python

import sys
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

def talker():
    pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped , queue_size=1)
    rospy.init_node('best_node_ever', anonymous=True)

    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = 0.35
    drive_msg.drive.speed = 2.5

    while not rospy.is_shutdown():
        pub.publish(drive_msg)
    

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
