#!/usr/bin/env python3
"""
Comprehensive servo verification script
This will test the servo reversal logic step by step with clear feedback
"""

import time
import sys
from servo_controller import ServoController

def test_raw_servo_movement():
    """Test raw servo movement without ServoController to verify hardware"""
    print("\n🔧 Testing Raw Servo Movement (Hardware Verification)")
    print("-" * 60)
    
    import RPi.GPIO as GPIO
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)  # Servo 1
        GPIO.setup(19, GPIO.OUT)  # Servo 2
        
        servo1_pwm = GPIO.PWM(18, 50)
        servo2_pwm = GPIO.PWM(19, 50)
        
        servo1_pwm.start(0)
        servo2_pwm.start(0)
        
        # Test 0 degrees (should be one extreme)
        print("Setting both servos to 0° (2.5% duty cycle)")
        servo1_pwm.ChangeDutyCycle(2.5)
        servo2_pwm.ChangeDutyCycle(2.5)
        time.sleep(2)
        
        # Test 180 degrees (should be other extreme)
        print("Setting both servos to 180° (12.5% duty cycle)")
        servo1_pwm.ChangeDutyCycle(12.5)
        servo2_pwm.ChangeDutyCycle(12.5)
        time.sleep(2)
        
        # Back to 0
        print("Back to 0°")
        servo1_pwm.ChangeDutyCycle(2.5)
        servo2_pwm.ChangeDutyCycle(2.5)
        time.sleep(2)
        
        servo1_pwm.stop()
        servo2_pwm.stop()
        GPIO.cleanup()
        
        print("✅ Raw servo test complete")
        print("📝 Observe: Did both servos move in the same direction?")
        
    except Exception as e:
        print(f"❌ Raw servo test failed: {e}")
        GPIO.cleanup()

def test_servo_controller_logic():
    """Test the ServoController logic with reversal"""
    print("\n🎯 Testing ServoController Logic (With Reversal)")
    print("-" * 60)
    
    servo_controller = ServoController()
    
    if not servo_controller.is_initialized:
        print("❌ ServoController failed to initialize")
        return
    
    try:
        print(f"Servo 1 reversal setting: {servo_controller.servo1_reversed}")
        print(f"Servo 2 reversal setting: {servo_controller.servo2_reversed}")
        
        print("\n🟡 Test 1: Setting both servos to 'CLOSED' position (0°)")
        print("Expected behavior:")
        print("  - Servo 1: Should move to 180° (reversed from 0°)")
        print("  - Servo 2: Should move to 0° (normal)")
        
        servo_controller.set_servo_angle(servo_controller.servo1_pwm, 0, 1)
        time.sleep(1)
        servo_controller.set_servo_angle(servo_controller.servo2_pwm, 0, 2)
        time.sleep(3)
        
        print("\n🟡 Test 2: Setting both servos to 'OPEN' position (180°)")
        print("Expected behavior:")
        print("  - Servo 1: Should move to 0° (reversed from 180°)")
        print("  - Servo 2: Should move to 180° (normal)")
        
        servo_controller.set_servo_angle(servo_controller.servo1_pwm, 180, 1)
        time.sleep(1)
        servo_controller.set_servo_angle(servo_controller.servo2_pwm, 180, 2)
        time.sleep(3)
        
        print("\n🟡 Test 3: Using drop payload methods")
        print("This should open both servos (release payloads)")
        
        print("Dropping payload 1 (Red)...")
        servo_controller.drop_payload_1()
        time.sleep(2)
        
        print("Dropping payload 2 (Blue)...")
        servo_controller.drop_payload_2()
        time.sleep(2)
        
        print("\n🟡 Test 4: Resetting servos")
        print("This should close both servos (secure payloads)")
        servo_controller.reset_servos()
        time.sleep(2)
        
        print("✅ ServoController logic test complete")
        
    except Exception as e:
        print(f"❌ ServoController test failed: {e}")
    finally:
        servo_controller.cleanup()

def main():
    print("🚁 Drone Servo Verification Script")
    print("=" * 60)
    print("This script will verify that:")
    print("1. Hardware servos are working")
    print("2. Servo 1 reversal logic is correct")
    print("3. Drop mechanisms work as expected")
    print("=" * 60)
    
    choice = input("\nChoose test:\n1. Raw servo test (hardware only)\n2. ServoController test (with reversal logic)\n3. Both tests\nEnter choice (1/2/3): ")
    
    if choice in ['1', '3']:
        test_raw_servo_movement()
        if choice == '3':
            input("\nPress Enter to continue to ServoController test...")
    
    if choice in ['2', '3']:
        test_servo_controller_logic()
    
    print("\n🎯 Test Summary:")
    print("After running these tests, verify:")
    print("✓ Servo 1 moves opposite to Servo 2 when given same angle")
    print("✓ 'Drop' commands actually open the payload mechanism")
    print("✓ 'Reset' commands close the payload mechanism")
    print("✓ Both servos respond reliably")
    
    print("\n🔧 If servos still don't work correctly:")
    print("1. Check wiring and power supply")
    print("2. Verify servo specifications (some may need different pulse widths)")
    print("3. Adjust servo1_reversed/servo2_reversed flags in servo_controller.py")
    print("4. Test with different angles if needed")

if __name__ == "__main__":
    main()
