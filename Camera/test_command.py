#!/usr/bin/env python

from drive2 import command_motors as cm
import test_bridge2
import rospy
import time

def main():
    for i in range(100):
        car.talker(0.0, 1.0)
    
    time.sleep(1)

    for i in range(100):
        car.talker(0.0, 0.0)

    time.sleep(1)

    for i in range(100):
        car.talker(0.0, -1.0)

    time.sleep(1)

    for i in range(100):
        car.talker(0.0, 0.0)

    time.sleep(1)
       
 
if __name__=='__main__':
    car = cm()
    while not rospy.is_shutdown():
        try:    
            main()
        except rospy.ROSInterruptException:
            pass


