#!/usr/bin/env python

import sys
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time


class racer():

    def __init__(self):
        self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped , queue_size=1)

        self.count = 0

    def set_speed(self, linear, angular):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angular
        drive_msg.drive.speed = linear
        self. pub.publish(drive_msg)
    

if __name__=='__main__':
    rospy.init_node('best_node_ever', anonymous=True)
    print("node initialized")
    car = racer()
    print("Sampling rate set to 100Hz")
    rate = rospy.Rate(500)
    print("speed set to 1.0")
    speed = 1.0

    while not rospy.is_shutdown():
        car.count = car.count + 1
        #rospy.loginfo(car.count)
        if (car.count == 20000):
           # print("speed increased")
            speed = 2.0
        elif (car.count == 40000):
           # print("Increasing")
            speed = 3.0
        elif (car.count == 60000):
          #  print("Breaking")
            speed = 2.0
        elif (car.count == 80000):
         #   print("Breaking")
            speed = 1.0
        elif (car.count == 100000):
        #    print("Stopping")
            speed = 0.0
        try:
            car.set_speed(speed, 0.0)
        except rospy.ROSInterruptException:
            pass
        print(car.count)
        rate.sleep()

        #)) Compensation on the servo to drive straight at different speeds.
        # At speed 1.0 servo = 0.005
        # At speed 2.0 servo = 0.0
        # At speed 3.0 servo slightly drifting right  
        # At speed 4.0 servo = 
        # At speed 5.0 servo = 
