#!/usr/bin/env python3
"""
Servo Test Script for Kuzgun 2025
This script allows you to test the servo functionality independently
"""

from servo_controller import ServoController
import time
import sys

def main():
    print("==========================================")
    print("Kuzgun 2025 Servo Test")
    print("==========================================")
    
    # Initialize servo controller
    print("Initializing servo controller...")
    servo_controller = ServoController(servo1_pin=18, servo2_pin=19)
    
    if not servo_controller.is_initialized:
        print("Failed to initialize servo controller!")
        sys.exit(1)
    
    print("Servo controller initialized successfully!")
    print("")
    
    try:
        while True:
            print("\nServo Test Menu:")
            print("1. Test Servo 1")
            print("2. Test Servo 2") 
            print("3. Test Both Servos")
            print("4. Drop Payload 1")
            print("5. Drop Payload 2")
            print("6. Drop Both Payloads")
            print("7. Reset Servos")
            print("8. Exit")
            
            choice = input("\nEnter your choice (1-8): ").strip()
            
            if choice == '1':
                print("Testing Servo 1...")
                servo_controller.set_servo_angle(servo_controller.servo1_pwm, -90)
                time.sleep(1)
                servo_controller.set_servo_angle(servo_controller.servo1_pwm, 0)
                print("Servo 1 test complete")
                
            elif choice == '2':
                print("Testing Servo 2...")
                servo_controller.set_servo_angle(servo_controller.servo2_pwm, 90)
                time.sleep(1)
                servo_controller.set_servo_angle(servo_controller.servo2_pwm, 0)
                print("Servo 2 test complete")
                
            elif choice == '3':
                servo_controller.test_servos()
                
            elif choice == '4':
                servo_controller.drop_payload_1()
                
            elif choice == '5':
                servo_controller.drop_payload_2()
                
            elif choice == '6':
                servo_controller.drop_both_payloads()
                
            elif choice == '7':
                servo_controller.reset_servos()
                
            elif choice == '8':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice! Please enter 1-8.")
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        
    except Exception as e:
        print(f"Error during testing: {e}")
        
    finally:
        servo_controller.cleanup()
        print("Servo test completed and cleanup done.")

if __name__ == "__main__":
    main()