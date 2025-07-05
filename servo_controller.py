import RPi.GPIO as GPIO
import time
import threading

class ServoController:
    def __init__(self, servo1_pin=18, servo2_pin=19):
        """
        Initialize servo controller for two servos
        
        Args:
            servo1_pin (int): GPIO pin for first servo - RED PAYLOAD (drops to blue targets)
            servo2_pin (int): GPIO pin for second servo - BLUE PAYLOAD (drops to red targets)
        """
        self.servo1_pin = servo1_pin
        self.servo2_pin = servo2_pin
        self.is_initialized = False
        
        # Servo angles
        self.closed_angle = 0    # Angle when payload is secured
        self.open_angle = 90     # Angle when payload is released
        
        # Initialize GPIO
        self.setup_gpio()
        
    def setup_gpio(self):
        """Setup GPIO pins and PWM for servos"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo1_pin, GPIO.OUT)
            GPIO.setup(self.servo2_pin, GPIO.OUT)
            
            # Create PWM instances (50Hz for servos)
            self.servo1_pwm = GPIO.PWM(self.servo1_pin, 50)
            self.servo2_pwm = GPIO.PWM(self.servo2_pin, 50)
            
            # Start PWM with 0% duty cycle
            self.servo1_pwm.start(0)
            self.servo2_pwm.start(0)
            
            # Set initial position (closed)
            self.set_servo_angle(self.servo1_pwm, self.closed_angle)
            self.set_servo_angle(self.servo2_pwm, self.closed_angle)
            
            self.is_initialized = True
            print("Servo controller initialized successfully")
            
        except Exception as e:
            print(f"Error initializing servo controller: {e}")
            self.is_initialized = False
    
    def angle_to_duty_cycle(self, angle):
        """
        Convert angle to duty cycle for servo control
        
        Args:
            angle (float): Angle in degrees (0-180)
            
        Returns:
            float: Duty cycle percentage
        """
        # Standard servo: 1ms (5% duty) = 0°, 2ms (10% duty) = 180°
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def set_servo_angle(self, pwm_instance, angle):
        """
        Set servo to specific angle
        
        Args:
            pwm_instance: PWM instance for the servo
            angle (float): Target angle in degrees
        """
        if not self.is_initialized:
            print("Servo controller not initialized")
            return
            
        try:
            duty_cycle = self.angle_to_duty_cycle(angle)
            pwm_instance.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)  # Give servo time to move
            pwm_instance.ChangeDutyCycle(0)  # Stop sending signal
            
        except Exception as e:
            print(f"Error setting servo angle: {e}")
    
    def drop_payload_1(self):
        """Drop RED payload from servo 1 (to blue target)"""
        print("Dropping RED payload (Servo 1) to blue target...")
        self.set_servo_angle(self.servo1_pwm, self.open_angle)
        
    def drop_payload_2(self):
        """Drop BLUE payload from servo 2 (to red target)"""
        print("Dropping BLUE payload (Servo 2) to red target...")
        self.set_servo_angle(self.servo2_pwm, self.open_angle)
        
    def drop_both_payloads(self):
        """Drop both payloads simultaneously"""
        print("Dropping both payloads...")
        
        # Use threading to move both servos simultaneously
        thread1 = threading.Thread(target=self.drop_payload_1)
        thread2 = threading.Thread(target=self.drop_payload_2)
        
        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
        
        print("Both payloads dropped successfully")
    
    def reset_servos(self):
        """Reset both servos to closed position"""
        print("Resetting servos to closed position...")
        self.set_servo_angle(self.servo1_pwm, self.closed_angle)
        self.set_servo_angle(self.servo2_pwm, self.closed_angle)
        print("Servos reset to closed position")
    
    def test_servos(self):
        """Test servo movement"""
        if not self.is_initialized:
            print("Cannot test servos - not initialized")
            return
            
        print("Testing servos...")
        
        # Test servo 1
        print("Testing servo 1...")
        self.set_servo_angle(self.servo1_pwm, self.open_angle)
        time.sleep(1)
        self.set_servo_angle(self.servo1_pwm, self.closed_angle)
        time.sleep(1)
        
        # Test servo 2
        print("Testing servo 2...")
        self.set_servo_angle(self.servo2_pwm, self.open_angle)
        time.sleep(1)
        self.set_servo_angle(self.servo2_pwm, self.closed_angle)
        time.sleep(1)
        
        print("Servo test completed")
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        if self.is_initialized:
            try:
                self.servo1_pwm.stop()
                self.servo2_pwm.stop()
                GPIO.cleanup()
                print("Servo controller cleanup completed")
            except Exception as e:
                print(f"Error during cleanup: {e}")
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()