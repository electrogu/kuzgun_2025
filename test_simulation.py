#!/usr/bin/env python3
"""
Test simulation script for Kuzgun 2025
Bu script İHA olmadan yazılımı test etmek için kullanılır
"""
    
import cv2
import time
import os
from camera_handler import CameraHandler
from image_processor import ImageProcessor
from servo_controller import ServoController
import numpy as np

# Test parametreleri
test_images = ["images/hexagon.jpg", "images/square.jpg", "images/triangle.jpg"]
simulated_altitude = 20  # metre
simulated_velocity = 15  # m/s

# Renk aralıkları (main.py'den kopyalandı)
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([130, 255, 255])

class MockVehicle:
    """Drone simülasyonu için sahte vehicle sınıfı"""
    def __init__(self):
        self.altitude = simulated_altitude
        self.velocity = simulated_velocity
    
    def get_altitude(self):
        return self.altitude
    
    def get_speed(self):
        return self.velocity

def test_with_images():
    """Test görüntüleri ile sistem testi"""
    print("=== Test Görüntüleri ile Sistem Testi ===")
    
    processor = ImageProcessor([(lower_red1, upper_red1), (lower_red2, upper_red2), (lower_blue, upper_blue)])
    
    for image_path in test_images:
        if os.path.exists(image_path):
            print(f"\nTest ediliyor: {image_path}")
            
            # Görüntüyü yükle
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"Görüntü yüklenemedi: {image_path}")
                continue
                
            frame = cv2.resize(frame, (1280, 720))
            
            # İşle
            mask = processor.process_frame(frame)
            largest_contour, area, center = processor.find_largest_contour(mask)
            
            if largest_contour is not None:
                detected_shape, num_vertices, vertices = processor.detect_shape(largest_contour)
                
                # Hedef rengi tespit et
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
                
                contour_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                cv2.fillPoly(contour_mask, [largest_contour], 255)
                
                red_overlap = cv2.bitwise_and(contour_mask, red_mask)
                blue_overlap = cv2.bitwise_and(contour_mask, blue_mask)
                
                red_pixels = cv2.countNonZero(red_overlap)
                blue_pixels = cv2.countNonZero(blue_overlap)
                
                target_color = "unknown"
                if red_pixels > blue_pixels and red_pixels > 100:
                    target_color = "red"
                elif blue_pixels > red_pixels and blue_pixels > 100:
                    target_color = "blue"
                
                # Sonuçları göster
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
                cv2.putText(frame, f"Shape: {detected_shape}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, f"Vertices: {num_vertices}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, f"Color: {target_color}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, f"Area: {area}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                print(f"  - Şekil: {detected_shape}")
                print(f"  - Köşe sayısı: {num_vertices}")
                print(f"  - Renk: {target_color}")
                print(f"  - Alan: {area}")
                print(f"  - Merkez: {center}")
            else:
                print("  - Hiçbir şekil tespit edilmedi")
                cv2.putText(frame, "No shape detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Görüntüyü göster
            cv2.imshow("Test Result", frame)
            cv2.waitKey(0)
    
    cv2.destroyAllWindows()

def test_servo_without_rpi():
    """Raspberry Pi olmadan servo testi (mock)"""
    print("\n=== Servo Testi (Simülasyon) ===")
    print("Not: Raspberry Pi bağlı değil, servo komutları simüle ediliyor")
    
    try:
        # Mock servo controller
        class MockServoController:
            def __init__(self):
                self.red_dropped = False
                self.blue_dropped = False
                print("Mock Servo Controller başlatıldı")
            
            def drop_payload_1(self):
                print("🔴 SERVO 1: Kırmızı payload bırakıldı (mavi hedefe)")
                self.red_dropped = True
            
            def drop_payload_2(self):
                print("🔵 SERVO 2: Mavi payload bırakıldı (kırmızı hedefe)")
                self.blue_dropped = True
            
            def reset_servos(self):
                print("🔄 Servolar sıfırlandı")
                self.red_dropped = False
                self.blue_dropped = False
            
            def test_servos(self):
                print("🔧 Servo testi yapılıyor...")
                time.sleep(1)
                print("✅ Servo testi tamamlandı")
            
            def cleanup(self):
                print("🧹 Mock servo temizlendi")
        
        servo = MockServoController()
        
        print("\nServo test menüsü:")
        print("1 - Payload 1 bırak")
        print("2 - Payload 2 bırak") 
        print("r - Sıfırla")
        print("t - Test")
        print("q - Çıkış")
        
        while True:
            key = input("\nKomut girin: ").strip().lower()
            
            if key == '1':
                servo.drop_payload_1()
            elif key == '2':
                servo.drop_payload_2()
            elif key == 'r':
                servo.reset_servos()
            elif key == 't':
                servo.test_servos()
            elif key == 'q':
                break
            else:
                print("Geçersiz komut!")
        
        servo.cleanup()
        
    except Exception as e:
        print(f"Mock servo test hatası: {e}")

def main():
    print("KUZGUN 2025 Test Simülasyonu")
    print("============================")
    
    while True:
        print("\nTest seçenekleri:")
        print("1 - Test görüntüleri ile algoritma testi")
        print("2 - Servo simülasyon testi")
        print("3 - Çıkış")
        
        choice = input("\nSeçiminizi yapın (1-3): ").strip()
        
        if choice == '1':
            test_with_images()
        elif choice == '2':
            test_servo_without_rpi()
        elif choice == '3':
            print("Test tamamlandı!")
            break
        else:
            print("Geçersiz seçim!")

if __name__ == "__main__":
    main()
