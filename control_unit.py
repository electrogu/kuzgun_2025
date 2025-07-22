import time
import cv2

import RPi.GPIO as GPIO

# Servo motorları kontrol eder ve hedef tespit edildiğinde payload drop işlemini tetikler.
class ControlUnit:

    # servo motorları başlatır
    def __init__(self, servo1_pin=17, servo2_pin=27):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT) # burayı bi soruştur. 17 mi?
        self.servo1 = GPIO.PWM(servo1_pin, 50)
        self.servo1.start(0)

        GPIO.setup(servo2_pin, GPIO.OUT)
        self.servo2 = GPIO.PWM(servo2_pin, 50)
        self.servo2.start(0)



    def check_and_trigger(self, contour):
        if contour is not None: # hedef tespit edildi mi?
            area = cv2.contourArea(contour) # alan hesabı
            if area > 500:    # 500 pikselden büyükse
                self.activate_servo()   # servo'yu tetikle          

    def activate_servo(self):
        print("Servo activated!")
        # Add servo control logic here

