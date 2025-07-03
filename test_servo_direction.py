#!/usr/bin/env python3
"""
Test script to verify servo direction reversal is working correctly
This will test both servos with clear indication of expected behavior
"""

import time
import sys
from servo_controller import ServoController

def main():
    print("🧪 Testing Servo Direction Control")
    print("=" * 50)
    
    # Initialize servo controller
    servo_controller = ServoController()
    
    if not servo_controller.is_initialized:
        print("❌ Failed to initialize servo controller")
        return
    
    try:
        print("\n🔧 Servo Configuration:")
        print(f"   Servo 1 (Red payload): Reversed = {servo_controller.servo1_reversed}")
        print(f"   Servo 2 (Blue payload): Reversed = {servo_controller.servo2_reversed}")
        
        print("\n📋 Test Plan:")
        print("   1. Both servos start in CLOSED position (payload secured)")
        print("   2. Test OPEN position (payload released)")
        print("   3. Return to CLOSED position")
        print("   4. With reversal, Servo 1 should move opposite to Servo 2")
        
        input("\nPress Enter to start test...")
        
        # Step 1: Ensure both servos are closed
        print("\n🏁 Step 1: Setting both servos to CLOSED position")
        servo_controller.reset_servos()
        time.sleep(2)
        
        # Step 2: Test opening servo 1
        print("\n🔴 Step 2: Testing Servo 1 (Red payload) - OPEN")
        print("   Expected: Servo should OPEN to release red payload")
        servo_controller.set_servo_angle(servo_controller.servo1_pwm, servo_controller.open_angle, 1)
        time.sleep(3)
        
        # Step 3: Close servo 1
        print("\n🔴 Step 3: Testing Servo 1 (Red payload) - CLOSE")
        print("   Expected: Servo should CLOSE to secure red payload")
        servo_controller.set_servo_angle(servo_controller.servo1_pwm, servo_controller.closed_angle, 1)
        time.sleep(3)
        
        # Step 4: Test opening servo 2
        print("\n🔵 Step 4: Testing Servo 2 (Blue payload) - OPEN")
        print("   Expected: Servo should OPEN to release blue payload")
        servo_controller.set_servo_angle(servo_controller.servo2_pwm, servo_controller.open_angle, 2)
        time.sleep(3)
        
        # Step 5: Close servo 2
        print("\n🔵 Step 5: Testing Servo 2 (Blue payload) - CLOSE")
        print("   Expected: Servo should CLOSE to secure blue payload")
        servo_controller.set_servo_angle(servo_controller.servo2_pwm, servo_controller.closed_angle, 2)
        time.sleep(3)
        
        # Step 6: Test both servos simultaneously
        print("\n⚡ Step 6: Testing both servos simultaneously")
        print("   Opening both servos (payload release)...")
        servo_controller.drop_both_payloads()
        time.sleep(3)
        
        print("\n   Closing both servos (reset position)...")
        servo_controller.reset_servos()
        time.sleep(2)
        
        print("\n✅ Test completed successfully!")
        print("\n📝 Verification checklist:")
        print("   ☐ Servo 1 moved in reverse direction (180° became 0°, 0° became 180°)")
        print("   ☐ Servo 2 moved normally")
        print("   ☐ Both servos properly opened and closed")
        print("   ☐ Payload release mechanism works as expected")
        
    except KeyboardInterrupt:
        print("\n⚠️ Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
    finally:
        print("\n🧹 Cleaning up...")
        servo_controller.cleanup()
        print("Test complete!")

if __name__ == "__main__":
    main()
