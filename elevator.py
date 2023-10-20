#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
import time
import drivers

GPIO.setwarnings(False)

display = drivers.Lcd()

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

floor_state = rospy.Publisher('Floor_state', String, queue_size=10)

# Motor pins (change these pins based on your connections)
motor_pin1 = 17  # Input 1 for motor driver
motor_pin2 = 27  # Input 2 for motor driver
PWM_pin = 18  # Enable pin for motor driver

# Setup GPIO pins for motor
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)
GPIO.setup(PWM_pin, GPIO.OUT)

pwm = GPIO.PWM(PWM_pin, 100)
pwm.start(0)

# Set the motor to run clockwise
def open():
    pwm.ChangeDutyCycle(100)
    GPIO.output(motor_pin1, GPIO.HIGH)
    GPIO.output(motor_pin2, GPIO.LOW)
    time.sleep(7.5)
    motor_stop()

# Set the motor to run counterclockwise
def close():
    pwm.ChangeDutyCycle(100)
    GPIO.output(motor_pin1, GPIO.LOW)
    GPIO.output(motor_pin2, GPIO.HIGH)
    time.sleep(7.5)
    motor_stop()

# Stop the motor
def motor_stop():
    pwm.ChangeDutyCycle(0)

# Change display text
def change_display(new_text):
    if new_text == 1:
        time.sleep(0.1)
        display.lcd_display_string("                #  ", 1)
        display.lcd_display_string("    #############  ", 2)
        display.lcd_display_string("    #           #  ", 3)
        display.lcd_display_string("     #          #  ", 4)
    elif new_text == 2:
        time.sleep(0.1)
        display.lcd_display_string("     ####       #  ", 1)
        display.lcd_display_string("    #    #      #  ", 2)
        display.lcd_display_string("    #     #     #  ", 3)
        display.lcd_display_string("     #     ######  ", 4)
    


def callback(data):
    global change_now

    if data.data == "1F_open":
        open()
        floor_state.publish("first_open_in")
        change_display(1)
        time.sleep(5)
    elif data.data == "1F_in":
        time.sleep(1)
        close()
        time.sleep(3)
        display.lcd_clear()
        change_display(2)
        time.sleep(3)
        open()
        floor_state.publish("second_open_out")
        time.sleep(13)
        close()

    if data.data == "2F_open":
        open()
        floor_state.publish("second_open_in")
        change_display(2)
        time.sleep(5)
    elif data.data == "2F_in":
        time.sleep(1)
        close()
        time.sleep(3)
        display.lcd_clear()
        change_display(1)
        time.sleep(3)
        open()
        floor_state.publish("first_open_out")
        time.sleep(13)
        close()

def dc_motor():
    rospy.init_node('dc_elevator', anonymous=True)
    rospy.Subscriber('elevator', String, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # When Ctrl+C is pressed, stop the motor and cleanup GPIO
        print("Cleaning up!")
        display.lcd_clear()        
        motor_stop()
        GPIO.cleanup()

if __name__ == "__main__":
    change_display(1)
    dc_motor()
