#!/usr/bin/env python3
"""
Comprehensive Servo Troubleshooting Script
Tests servo connections and identifies common issues
"""
import RPi.GPIO as GPIO
import time
import sys

def test_servo_basic():
    """Basic servo test with detailed diagnostics"""
    print("=" * 50)
    print("SERVO TROUBLESHOOTING DIAGNOSTIC")
    print("=" * 50)
    
    # Pin configuration
    servo1_pin = 18
    servo2_pin = 19
    
    print(f"Testing Servo 1 on GPIO {servo1_pin}")
    print(f"Testing Servo 2 on GPIO {servo2_pin}")
    print()
    
    try:
        # Setup GPIO
        print("1. Setting up GPIO...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup pins
        print("2. Configuring pins as OUTPUT...")
        GPIO.setup(servo1_pin, GPIO.OUT)
        GPIO.setup(servo2_pin, GPIO.OUT)
        
        # Create PWM instances
        print("3. Creating PWM instances (50Hz)...")
        servo1_pwm = GPIO.PWM(servo1_pin, 50)
        servo2_pwm = GPIO.PWM(servo2_pin, 50)
        
        # Start PWM
        print("4. Starting PWM...")
        servo1_pwm.start(0)
        servo2_pwm.start(0)
        
        print("5. Testing servo movements...")
        print()
        
        # Test function
        def move_servo(pwm, name, angle):
            duty_cycle = 2.5 + (angle / 180.0) * 10.0
            print(f"   Moving {name} to {angle}° (duty cycle: {duty_cycle:.2f}%)")
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(1.0)  # Give servo time to move
            pwm.ChangeDutyCycle(0)  # Stop sending signal
            time.sleep(0.5)
        
        # Extended test sequence
        print("Testing Servo 1 (Red Payload):")
        move_servo(servo1_pwm, "Servo 1", 0)
        move_servo(servo1_pwm, "Servo 1", 45)
        move_servo(servo1_pwm, "Servo 1", 90)
        move_servo(servo1_pwm, "Servo 1", 135)
        move_servo(servo1_pwm, "Servo 1", 180)
        move_servo(servo1_pwm, "Servo 1", 90)
        move_servo(servo1_pwm, "Servo 1", 0)
        
        print("\nTesting Servo 2 (Blue Payload):")
        move_servo(servo2_pwm, "Servo 2", 0)
        move_servo(servo2_pwm, "Servo 2", 45)
        move_servo(servo2_pwm, "Servo 2", 90)
        move_servo(servo2_pwm, "Servo 2", 135)
        move_servo(servo2_pwm, "Servo 2", 180)
        move_servo(servo2_pwm, "Servo 2", 90)
        move_servo(servo2_pwm, "Servo 2", 0)
        
        print("\nTesting both servos simultaneously:")
        print("   Moving both to 90°...")
        servo1_pwm.ChangeDutyCycle(7.5)  # 90 degrees
        servo2_pwm.ChangeDutyCycle(7.5)  # 90 degrees
        time.sleep(2)
        
        print("   Moving both to 0°...")
        servo1_pwm.ChangeDutyCycle(2.5)  # 0 degrees
        servo2_pwm.ChangeDutyCycle(2.5)  # 0 degrees
        time.sleep(2)
        
        # Stop PWM
        servo1_pwm.ChangeDutyCycle(0)
        servo2_pwm.ChangeDutyCycle(0)
        
        print("\n✅ Test completed successfully!")
        print("If servos didn't move, check the troubleshooting guide below.")
        
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
        return False
        
    finally:
        try:
            servo1_pwm.stop()
            servo2_pwm.stop()
        except:
            pass
        GPIO.cleanup()
        print("🧹 GPIO cleanup completed")
    
    return True

def print_troubleshooting_guide():
    """Print comprehensive troubleshooting guide"""
    print("\n" + "=" * 50)
    print("TROUBLESHOOTING GUIDE")
    print("=" * 50)
    
    print("\n🔍 IF SERVOS DON'T MOVE, CHECK:")
    print("\n1. POWER CONNECTIONS:")
    print("   ✓ Red wire → 5V (Pin 2 or 4)")
    print("   ✓ Black/Brown wire → GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)")
    print("   ✓ All connections are secure")
    
    print("\n2. SIGNAL CONNECTIONS:")
    print("   ✓ Servo 1 signal (Yellow/White) → GPIO 18 (Pin 12)")
    print("   ✓ Servo 2 signal (Yellow/White) → GPIO 19 (Pin 35)")
    
    print("\n3. POWER SUPPLY:")
    print("   ✓ Check if Pi's 5V can supply enough current")
    print("   ✓ Try external 5V power supply (2A+)")
    print("   ✓ Ensure common ground between Pi and external supply")
    
    print("\n4. SERVO TYPE:")
    print("   ✓ Standard servos (SG90, MG90S) should work")
    print("   ✓ Some servos need different signal ranges")
    print("   ✓ Continuous rotation servos behave differently")
    
    print("\n5. PHYSICAL CHECK:")
    print("   ✓ Servo isn't mechanically stuck")
    print("   ✓ No loose connections")
    print("   ✓ Servo LED (if any) indicates power")
    
    print("\n🔧 ADVANCED DIAGNOSTICS:")
    print("   • Use multimeter to check 5V presence")
    print("   • Use oscilloscope to verify PWM signal")
    print("   • Test with known working servo")
    print("   • Check Pi's 5V current capability")

def check_gpio_permissions():
    """Check if user has GPIO permissions"""
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.cleanup()
        return True
    except Exception as e:
        print(f"❌ GPIO Permission Error: {e}")
        print("💡 Try running with: sudo python3 servo_troubleshoot.py")
        return False

def main():
    print("🤖 Kuzgun 2025 Servo Diagnostic Tool")
    print("This will help identify servo connection issues\n")
    
    # Check permissions
    if not check_gpio_permissions():
        return
    
    # Run the test
    success = test_servo_basic()
    
    # Print troubleshooting guide
    print_troubleshooting_guide()
    
    if success:
        print("\n" + "=" * 50)
        print("TEST RESULTS:")
        print("✅ Script ran without errors")
        print("❓ Did you see servo movement?")
        print("   - YES: Servos are working correctly!")
        print("   - NO: Check troubleshooting guide above")
        print("=" * 50)
    
    print("\n🎯 NEXT STEPS:")
    print("1. If servos work here, the issue is in main code")
    print("2. If servos don't work, follow troubleshooting guide")
    print("3. Test manual drops with keys '1' and '2' in main program")

if __name__ == "__main__":
    main()
