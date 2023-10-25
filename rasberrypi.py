#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32, String
import time

prev_data = ""

servo_pin1 = 17
servo_pin2 = 27

pub = rospy.Publisher('led_control', Int32, queue_size=10)
pubpub = rospy.Publisher('ros_to_firebase', String, queue_size=10)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# GPIO.setup(servo_pin1, GPIO.OUT)
GPIO.setup(servo_pin2, GPIO.OUT)

# pwm1 = GPIO.PWM(servo_pin1, 50)
pwm2 = GPIO.PWM(servo_pin2, 50)
# pwm1.start(0)
pwm2.start(0)





# def servo_angle1(angle):
#     duty = angle / 18 + 2

#     GPIO.output(servo_pin1, True)
#     pwm1.ChangeDutyCycle(duty)
#     time.sleep(1)

def servo_angle2(angle):
    duty = angle / 18 + 2

    GPIO.output(servo_pin2, True)
    pwm2.ChangeDutyCycle(duty)
    time.sleep(1)

def callback(data):

    
    if data.data == "First_Open":
        rospy.loginfo(data)
        # servo_angle1(90)
        time.sleep(1)
        pub.publish(0)  

        
    if data.data == "First_Close":
        rospy.loginfo(data)
        pub.publish(1)
        time.sleep(0.1)

    if data.data == "Second_Open":
        rospy.loginfo(data)
        servo_angle2(90)
        time.sleep(1)
        pub.publish(2)  

        
    if data.data == "Second_Close":
        rospy.loginfo(data)
        pub.publish(3)
        time.sleep(0.1)
        
    if data.data == "Dust_ON":
        rospy.loginfo(data)
        pub.publish(4)
        time.sleep(2)
        
    if data.data == "Dust_OFF":
        rospy.loginfo(data)
        pub.publish(5)
        time.sleep(2)

    if data.data == "Hotel_Mode":
        rospy.loginfo(data)
        pub.publish(6)
        
    if data.data in ["101_go", "102_go", "201_go", "202_go"]:
        rospy.loginfo(data)
        pub.publish(7)
        
    if data.data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
        rospy.loginfo(data)
        pub.publish(8)
        
    if data.data == "STANBY":
        rospy.loginfo(data)
        pub.publish(9)

    if data.data == "home_ARRIVE":
        rospy.loginfo(data)
        pub.publish(10)
def callback2(data):
    global prev_data
    
    if data.data != prev_data:  
        if data.data == "Lock1_done":
            if prev_data != "Lock1_done":
                time.sleep(0.5)
                rospy.loginfo("Lock1")
                # servo_angle1(0)
                time.sleep(0.1)
        if data.data == "Lock2_done":
            if prev_data != "Lock2_done":
                time.sleep(0.5)
                rospy.loginfo("Lock2")
                servo_angle2(0)
                time.sleep(0.1)
        pubpub.publish(data.data)
        rospy.loginfo('I heard %s', data.data)
    
    prev_data = data.data

        
def servo_motor():
    rospy.init_node('servo', anonymous=True)
    rospy.Subscriber('firebase_to_ros', String, callback)
    rospy.Subscriber('state', String, callback2)
    rospy.spin()

try:
    servo_motor()
except KeyboardInterrupt:
    pwm1.stop()
    GPIO.cleanup()
