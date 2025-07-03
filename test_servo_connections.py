#!/usr/bin/env python3
"""
Simple servo connection test
"""
import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
servo1_pin = 18
servo2_pin = 19

GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

# Create PWM instances
servo1 = GPIO.PWM(servo1_pin, 50)  # 50Hz
servo2 = GPIO.PWM(servo2_pin, 50)  # 50Hz

servo1.start(0)
servo2.start(0)

def set_servo_angle(servo, angle):
    duty_cycle = 2.5 + (angle / 180.0) * 10.0
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

try:
    print("Testing Servo 1 (GPIO 18)...")
    set_servo_angle(servo1, 0)    # 0 degrees
    time.sleep(1)
    set_servo_angle(servo1, 90)   # 90 degrees
    time.sleep(1)
    set_servo_angle(servo1, 0)    # back to 0
    
    print("Testing Servo 2 (GPIO 19)...")
    set_servo_angle(servo2, 0)    # 0 degrees
    time.sleep(1)
    set_servo_angle(servo2, 90)   # 90 degrees
    time.sleep(1)
    set_servo_angle(servo2, 0)    # back to 0
    
    print("Both servos tested successfully!")
    
except KeyboardInterrupt:
    print("Test interrupted")
    
finally:
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
    print("Cleanup complete")
