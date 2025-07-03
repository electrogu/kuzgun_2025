import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)  # Servo 1
GPIO.setup(19, GPIO.OUT)  # Servo 2

servo1 = GPIO.PWM(18, 50)
servo2 = GPIO.PWM(19, 50)

servo1.start(7.5)  # 90 degrees
servo2.start(7.5)  # 90 degrees

time.sleep(2)

servo1.stop()
servo2.stop()
GPIO.cleanup()

print("Test complete - servos should have moved to 90 degrees")
