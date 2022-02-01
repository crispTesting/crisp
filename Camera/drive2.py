#!/usr/bin/env python

import sys
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

class command_motors:

    def __init__(self):
        self.motor_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped , queue_size=1)
        self.vel_sub = rospy.Subscriber('/random_just_random', Float32 , self.vel_callback)
        self.speed = 0

    def vel_callback(self, msg):
        self.speed = msg.data
        rospy.loginfo(str(self.speed))
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = self.speed
        self.motor_pub.publish(drive_msg)



def main(args):
    rospy.init_node('command_motors', anonymous=True)
    cm = command_motors()
    rospy.spin()
        


if __name__=='__main__':
    main(sys.argv)
