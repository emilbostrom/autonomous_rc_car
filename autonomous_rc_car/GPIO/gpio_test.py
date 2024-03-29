#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

BUTTON_GPIO = 16
MOTOR_PWM_1_PIN = 21
MOTOR_PWM_2_PIN = 18
MOTOR_ENABLE_PIN = 25
DC_MOTOR_OFF = 0

if __name__ == '__main__':
    rospy.init_node('button_state_publisher')

    pub = rospy.Publisher('button_state', Bool, queue_size = 10)

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(MOTOR_ENABLE_PIN, GPIO.OUT)
    GPIO.output(MOTOR_ENABLE_PIN, GPIO.HIGH)

    print(f"Enable pin mode: {GPIO.gpio_function(MOTOR_ENABLE_PIN)}")

    time.sleep(10)

    dc = 50.0
    freq = 2e3
    GPIO.setup(MOTOR_PWM_1_PIN, GPIO.OUT)
    print(f"PWM 1 pin mode: {GPIO.gpio_function(MOTOR_PWM_1_PIN)}")
    pwm_1 = GPIO.PWM(MOTOR_PWM_1_PIN, freq)

    GPIO.setup(MOTOR_PWM_2_PIN, GPIO.OUT)
    print(f"PWM 2 pin mode: {GPIO.gpio_function(MOTOR_PWM_2_PIN)}")
    pwm_2 = GPIO.PWM(MOTOR_PWM_2_PIN, freq)

    pwm_1.start(dc)   # where dc is the duty cycle (0.0 <= dc <= 100.0)
    print("Started in1")
    pwm_2.start(DC_MOTOR_OFF)
    print("Started in2")

    time.sleep(5)

    pwm_1.ChangeDutyCycle(DC_MOTOR_OFF)  
    pwm_2.ChangeDutyCycle(dc)

    pwm_1.stop()
    pwm_2.stop()

    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        gpio_state = not GPIO.input(BUTTON_GPIO)
        pub.publish(gpio_state)
        rate.sleep()

    GPIO.cleanup()