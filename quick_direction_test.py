#!/usr/bin/env python3
"""
Quick verification that servo direction fix is working
Run this on your Raspberry Pi to test the servo direction correction
"""

from servo_controller import ServoController
import time

def main():
    print("🔧 Quick Servo Direction Test")
    print("=" * 40)
    
    servo_controller = ServoController()
    
    if not servo_controller.is_initialized:
        print("❌ Servo initialization failed")
        return
    
    print(f"✅ Servos initialized")
    print(f"   Servo 1 reversed: {servo_controller.servo1_reversed}")
    print(f"   Servo 2 reversed: {servo_controller.servo2_reversed}")
    
    print("\n🎯 Testing servo directions...")
    
    # Test 1: Close both (secure payloads)
    print("\n1. CLOSING both servos (secure payloads)")
    servo_controller.reset_servos()
    time.sleep(3)
    
    # Test 2: Open both (release payloads) 
    print("\n2. OPENING both servos (release payloads)")
    servo_controller.drop_both_payloads()
    time.sleep(3)
    
    # Test 3: Close again
    print("\n3. CLOSING both servos again")
    servo_controller.reset_servos()
    time.sleep(2)
    
    print("\n✅ Test complete!")
    print("\n📋 Verify that:")
    print("   - Servo 1 moved OPPOSITE to its previous direction")
    print("   - 'OPENING' actually releases the payload mechanism")
    print("   - 'CLOSING' actually secures the payload mechanism")
    print("   - Both servos work reliably")
    
    servo_controller.cleanup()

if __name__ == "__main__":
    main()
