#!/usr/bin/env python

####                                                                              ####
#  This is a simple test script for controlling the motor and servo of the car.      #
#                                                                                    #
#   This should be called from another script, when implementing for driving the car #                          
####                                                                              ####
 
import sys
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

                                                                       
def talker():
    # Creating a publisher which publishes the control commands onto the ROS topic /vesc/low_level/ackermann_cmd_mux/input/navigation
    pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped , queue_size=1)
    # Initiating the node
    rospy.init_node('best_node_ever', anonymous=True)

    # The ackerman message needed to send control signals to motor and servo.
    drive_msg = AckermannDriveStamped()
    # Servo control, ranges between - x -> + x, 0 = no steering. 
    drive_msg.drive.steering_angle = 0
    # Linear velocity, ranges -x -> + x, 0.0 = no velocity.
    drive_msg.drive.speed = 1.0

    # As long as the script is not shutdown it will continuosly publish control commands to the topic
    while not rospy.is_shutdown():
        pub.publish(drive_msg)
    

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
