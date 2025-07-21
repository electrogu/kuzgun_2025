"""
Mock Vehicle class for testing without actual drone connection
Bu sınıf gerçek drone olmadan test yapmanızı sağlar
"""

import time
import random

class Vehicle:
    def __init__(self, connection_string="/dev/ttyACM0", baud=57600, mock_mode=True):
        self.mock_mode = mock_mode
        
        if mock_mode:
            print("🚁 Mock Vehicle Mode: Drone simülasyonu aktif")
            self.mock_altitude = 20.0  # Test yüksekliği
            self.mock_speed = 15.0     # Test hızı
            self._init_mock_vehicle()
        else:
            # Gerçek drone bağlantısı
            try:
                from dronekit import connect
                self.vehicle = connect(connection_string, baud=baud, wait_ready=True)
                self.vehicle.wait_ready('autopilot_version')
                print(f"Connected to vehicle on: {connection_string}")
            except Exception as e:
                print(f"Gerçek drone bağlantısı başarısız: {e}")
                print("Mock mode'a geçiliyor...")
                self.mock_mode = True
                self._init_mock_vehicle()

    def _init_mock_vehicle(self):
        """Mock vehicle parametrelerini başlat"""
        self.start_time = time.time()
        print("Mock vehicle başlatıldı - test verileri kullanılıyor")

    def get_altitude(self):
        """Yükseklik bilgisi al (metre)"""
        if self.mock_mode:
            # Gerçekçi yükseklik simülasyonu (hafif dalgalanma ile)
            base_altitude = self.mock_altitude
            variation = random.uniform(-1.0, 1.0)  # ±1 metre dalgalanma
            return base_altitude + variation
        else:
            return self.vehicle.location.global_relative_frame.alt
    
    def get_speed(self):
        """Hız bilgisi al (m/s)"""
        if self.mock_mode:
            # Gerçekçi hız simülasyonu
            base_speed = self.mock_speed
            variation = random.uniform(-2.0, 2.0)  # ±2 m/s dalgalanma
            return max(0, base_speed + variation)  # Negatif hız olmasın
        else:
            return self.vehicle.groundspeed
    
    def get_gps(self):
        """GPS koordinatları al"""
        if self.mock_mode:
            # Mock GPS koordinatları (Gebze Teknik Üniversitesi yakını)
            return {
                'lat': 40.7674 + random.uniform(-0.001, 0.001),
                'lon': 29.4278 + random.uniform(-0.001, 0.001),
                'alt': self.get_altitude()
            }
        else:
            return self.vehicle.location.global_frame
    
    def set_mock_parameters(self, altitude=None, speed=None):
        """Mock parametrelerini güncelle"""
        if self.mock_mode:
            if altitude is not None:
                self.mock_altitude = altitude
                print(f"Mock altitude güncellendi: {altitude}m")
            if speed is not None:
                self.mock_speed = speed
                print(f"Mock speed güncellendi: {speed} m/s")
        else:
            print("Gerçek drone modunda mock parametreler güncellenemez")
    
    def get_flight_info(self):
        """Uçuş bilgilerini toplu al"""
        return {
            'altitude': self.get_altitude(),
            'speed': self.get_speed(),
            'gps': self.get_gps(),
            'mode': 'MOCK' if self.mock_mode else 'REAL'
        }
