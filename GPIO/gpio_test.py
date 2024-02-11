#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

BUTTON_GPIO = 16

if __name__ == '__main__':
    rospy.init_node('button_state_publisher')

    pub = rospy.Publisher('button_state', Bool, queue_size = 10)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        gpio_state = not GPIO.input(BUTTON_GPIO)
        pub.publish(gpio_state)
        rate.sleep()

    GPIO.cleanup()