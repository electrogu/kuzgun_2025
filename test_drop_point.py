#!/usr/bin/env python3
"""
Test script for drop point calculation
"""

from main import calculate_drop_point, aircraft_position
import main

print("=== DROP POINT CALCULATION TEST ===")
print(f"Aircraft position: {aircraft_position}")
print(f"Image dimensions: {main.image_width} x {main.image_height}")
print(f"Max distance: {main.max_distance:.2f}m")

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
