#!/usr/bin/env python3
"""
Simple drop point calculation test - no hardware dependencies
"""

import math

# Test constants (main.py'den kopyalandı)
g = 9.80665
resolution = (1280, 720)
image_width = resolution[0]
image_height = resolution[1]
FOV_Y = 41
FOV_X = 66
camera_height = 10
rotated_degree = 15
max_distance_back = camera_height * math.tan(math.radians(rotated_degree))
max_distance_front = camera_height * math.tan(math.radians(FOV_X-rotated_degree))
max_distance = max_distance_front + max_distance_back
max_width = 2 * camera_height * math.tan(math.radians(FOV_Y/2))
aircraft_position = ((image_width*(max_distance_back/(max_distance_back+max_distance_front))), image_height // 2)

TEST_MODE = True

def calculate_drop_point(aircraft_position, velocity, altitude):
    # Sıfır değerleri için güvenlik kontrolü (hesaplamadan ÖNCE)
    if velocity == 0 or velocity is None:
        velocity = 2.0  # Minimum test hızı
        print(f"⚠️  Velocity sıfır veya None - {velocity} m/s kullanılıyor")
    
    if altitude == 0 or altitude is None:
        altitude = 2.0  # Minimum test yüksekliği  
        print(f"⚠️  Altitude sıfır veya None - {altitude} m kullanılıyor")
    
    # Fizik hesaplamaları (düzeltilmiş değerlerle)
    time_to_fall = (2 * altitude / g) ** 0.5
    drop_distance = velocity * time_to_fall
    
    # Piksel koordinatlarına dönüştürme
    # drop_distance'ı metre cinsinden piksel cinsine çevir
    drop_distance_pixels = drop_distance * (image_width / max_distance)
    
    drop_x = int(aircraft_position[0] + drop_distance_pixels)
    drop_y = int(aircraft_position[1])
    
    # Debug bilgisi (test modunda)
    if TEST_MODE:
        print(f"🎯 Drop Point: V={velocity:.1f}m/s, H={altitude:.1f}m, Fall_time={time_to_fall:.2f}s, Distance={drop_distance:.2f}m, Pixels={drop_distance_pixels:.1f}")
    
    return (drop_x, drop_y)

print("=== DROP POINT CALCULATION TEST ===")
print(f"Aircraft position: {aircraft_position}")
print(f"Image dimensions: {image_width} x {image_height}")
print(f"Max distance: {max_distance:.2f}m")

# Test 1: Normal değerler
print('\n1️⃣ Test 1: Normal değerler (15 m/s, 20m)')
result = calculate_drop_point(aircraft_position, 15.0, 20.0)
print(f'Sonuç: {result}')

# Test 2: Sıfır velocity
print('\n2️⃣ Test 2: Sıfır velocity (0 m/s, 20m)')  
result = calculate_drop_point(aircraft_position, 0, 20.0)
print(f'Sonuç: {result}')

# Test 3: Sıfır altitude
print('\n3️⃣ Test 3: Sıfır altitude (15 m/s, 0m)')
result = calculate_drop_point(aircraft_position, 15.0, 0)
print(f'Sonuç: {result}')

# Test 4: Her ikisi sıfır
print('\n4️⃣ Test 4: Her ikisi sıfır (0 m/s, 0m)')
result = calculate_drop_point(aircraft_position, 0, 0)
print(f'Sonuç: {result}')

# Test 5: None değerler
print('\n5️⃣ Test 5: None değerler')
result = calculate_drop_point(aircraft_position, None, None)
print(f'Sonuç: {result}')

print("\n✅ Test tamamlandı!")
