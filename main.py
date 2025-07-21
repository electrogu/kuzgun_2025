#!/usr/bin/env python3

from camera_handler import CameraHandler
from image_processor import ImageProcessor
from servo_controller import ServoController
import cv2
import math
import numpy as np
import os

# Test modu için vehicle import'u
TEST_MODE = True  # Bu değişkeni False yapın gerçek drone ile test ederken

if TEST_MODE:
    from vehicle_mock import Vehicle
    print("🔧 TEST MODE: Mock vehicle kullanılıyor")
else:
    from vehicle import Vehicle
    print("🚁 PRODUCTION MODE: Gerçek vehicle kullanılıyor")

#            H    S    V

# Lower red range (hue 0-10)
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])

# Upper red range (hue 170-180)
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

#lower_red = [0, 120, 70]
#upper_red = [10, 255, 255]
# lower_red = [161, 155, 84]
# upper_red = [179, 255, 255]

#more accurate range
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([130, 255, 255])

resolution = (1280, 720)
camera_index = 0
g = 9.80665
image_width = resolution[0] #640  # Kamera çözünürlüğü (piksel cinsinden)
image_height = resolution[1] #480
FOV_Y = 41  # Horizontal field of view (degrees)
FOV_X = 66  # Horizontal field of view (degrees)
camera_height = 10 # altitude  # Kamera yüksekliği (metre cinsinden)
rotated_degree = 15
max_distance_back = camera_height * math.tan(math.radians(rotated_degree))  # Kameranın görebileceği maksimum mesafe, kamera hizasında arka taraf (metre cinsinden)
max_distance_front = camera_height * math.tan(math.radians(FOV_X-rotated_degree))  # Kameranın görebileceği maksimum mesafe, kamera hizasında ön taraf (metre cinsinden)
max_distance = max_distance_front + max_distance_back  #camera_height * math.tan(math.radians(FOV_X))  # Kameranın görebileceği maksimum mesafe, toplam (metre cinsinden)
max_width = 2 * camera_height * math.tan(math.radians(FOV_Y/2))  # Kameranın görebileceği maksimum genişlik (metre cinsinden)
aircraft_position = ((image_width*(max_distance_back/(max_distance_back+max_distance_front))), image_height // 2) # uçak konumu

# doküman bilgileri
sensor_width = 0.00645 # m
sensor_height = 0.00363 # m
focal_length = 0.00474 # m

# hesaplamalar, formüller
# (x) gsd = (sensor_width * altitude) / (image_width * focal_length)
# (y) gsd = (sensor_height * altitude) / (image_height * focal_length)
# real_area = (gsd_x * gsd_y) * contour_area

def main():
    # Initialize components
    # Test görüntüsü kullanmak isterseniz aşağıdaki satırı uncomment edin ve image_path ekleyin
    # camera = CameraHandler(image_path="images/square.jpg", resolution=resolution)
    
    # Gerçek kamera için:
    camera = CameraHandler(camera_index=camera_index, resolution=resolution)
    
    processor = ImageProcessor([(lower_red1, upper_red1), (lower_red2, upper_red2), (lower_blue, upper_blue)]) # , (lower_blue, upper_blue)
    
    # Initialize servo controller with safety check for Raspberry Pi
    servo_controller = None
    try:
        if TEST_MODE:
            # Mock servo controller for testing without RPi
            class MockServoController:
                def __init__(self):
                    self.red_dropped = False
                    self.blue_dropped = False
                    print("🔧 Mock Servo Controller başlatıldı (RPi gerekmiyor)")
                
                def drop_payload_1(self):
                    print("🔴 MOCK: Servo 1 - Kırmızı payload bırakıldı (mavi hedefe)")
                    self.red_dropped = True
                
                def drop_payload_2(self):
                    print("🔵 MOCK: Servo 2 - Mavi payload bırakıldı (kırmızı hedefe)")
                    self.blue_dropped = True
                
                def reset_servos(self):
                    print("🔄 MOCK: Servolar sıfırlandı")
                    self.red_dropped = False
                    self.blue_dropped = False
                
                def test_servos(self):
                    print("🔧 MOCK: Servo testi simüle ediliyor...")
                
                def drop_both_payloads(self):
                    print("🔄 MOCK: Her iki payload da bırakıldı")
                    
                def cleanup(self):
                    print("🧹 Mock servo temizlendi")
            
            servo_controller = MockServoController()
        else:
            servo_controller = ServoController(servo1_pin=18, servo2_pin=19)
    except Exception as e:
        print(f"Servo controller başlatılamadı: {e}")
        print("Mock servo controller kullanılıyor...")
        servo_controller = MockServoController()
    
    # Variables for payload drop control
    red_payload_dropped = False    # Servo 1 - Red payload (drops to blue target) / this can change
    blue_payload_dropped = False   # Servo 2 - Blue payload (drops to red target) / this can change

    cv2.namedWindow("GTU KUZGUN", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("GTU KUZGUN", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:

            frame = camera.get_frame()
            mask = processor.process_frame(frame)
            largest_contour, area, center = processor.find_largest_contour(mask)
            
            # Detect target color separately for each color range
            red_mask1 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_red1, upper_red1)
            red_mask2 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            blue_mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_blue, upper_blue)
                        
            # Determine which color target we're currently detecting
            target_color = "unknown"
            if largest_contour is not None:
                # Check if the largest contour belongs to red or blue
                contour_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                cv2.fillPoly(contour_mask, [largest_contour], 255)
                
                red_overlap = cv2.bitwise_and(contour_mask, red_mask)
                blue_overlap = cv2.bitwise_and(contour_mask, blue_mask)
                
                red_pixels = cv2.countNonZero(red_overlap)
                blue_pixels = cv2.countNonZero(blue_overlap)
                
                if red_pixels > blue_pixels and red_pixels > 100:  # Minimum pixel threshold
                    target_color = "red"
                elif blue_pixels > red_pixels and blue_pixels > 100:
                    target_color = "blue"
            
            velocity, altitude = get_drone_info()
            display_flight_info(frame, altitude, velocity)

            estimated_drop_point = calculate_drop_point(aircraft_position, velocity, altitude)
            show_estimated_drop_point(frame, estimated_drop_point[0], estimated_drop_point[1])

            gsd_x = (altitude * sensor_width) / (image_width * focal_length) #gsd değerini x için hesaplıyoruz
            gsd_y = (altitude * sensor_height) / (image_height * focal_length)  #gsd değerini y için hesaplıyoruz
            real_area = (gsd_x * gsd_y) * area # hesapladığımız gsd değerlerini ve ekrandaki pixel alanı çarparak gerçek alanı buluyoruz (yaklaşık olarak)
            
            if largest_contour is not None:
                
                detected_shape, num_vertices, vertices = processor.detect_shape(largest_contour)
                x, y, w, h = cv2.boundingRect(largest_contour)
                aspect_ratio = float(w/h) # elde ettiğimiz karenin yükseklik ve genişliğini oranlayarak alttaki satırda kare olup olmadığını kontrol ediyoruz yaklaşık olarak
                
                control = aspect_ratio >= 0.9 and aspect_ratio <= 1.1
                drop = estimated_drop_point
                dx, dy = drop
                cx, cy = center
                print(estimated_drop_point, center, area, real_area, 5 >= real_area >= 3, control, num_vertices, target_color) # Added target_color to debug output
                common_control = (dx - 15) <= cx <= (dx + 15) and control and num_vertices == 4
                
                # Drop logic based on target color
                if (common_control and (5 >= real_area >= 3 or 18 >= real_area >= 14)):
                    if target_color == "blue" and not red_payload_dropped:
                        print("Blue target detected - dropping RED payload (Servo 1)")
                        servo_controller.drop_payload_1()
                        red_payload_dropped = True
                    elif target_color == "red" and not blue_payload_dropped:
                        print("Red target detected - dropping BLUE payload (Servo 2)")
                        servo_controller.drop_payload_2()
                        blue_payload_dropped = True
                    elif target_color == "blue" and red_payload_dropped:
                        print("Blue target detected but red payload already dropped")
                    elif target_color == "red" and blue_payload_dropped:
                        print("Red target detected but blue payload already dropped")
                    elif target_color == "unknown":
                        print("Target color unknown - no payload drop")
                
                if ( (5 >= real_area >= 3 or 17 >= real_area >= 15) and num_vertices == 4): # hata payı +-1
                    # Draw contour with color based on target type
                    if target_color == "red":
                        cv2.drawContours(frame, [largest_contour], -1, (0, 0, 255), 3)  # Red contour for red target
                        cv2.putText(frame, "RED TARGET", (center[0]-50, center[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    elif target_color == "blue":
                        cv2.drawContours(frame, [largest_contour], -1, (255, 0, 0), 3)  # Blue contour for blue target
                        cv2.putText(frame, "BLUE TARGET", (center[0]-50, center[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    else:
                        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)  # Green contour for unknown
                        
                    cv2.circle(frame, center, 5, (0, 255, 255), -1)
                    # Draw the vertices on the frame
                    for vertex in vertices:
                        cv2.circle(frame, tuple(vertex), 5, (255, 255, 255), -1)
                

                # Düşüş noktası ve hedef arasındaki mesafeyi hesapla
                distance = calculate_distance(estimated_drop_point, center)

                # Düşüş noktası ve hedef arasındaki mesafeyi ekrana yazdır
                cv2.putText(frame, f"Distance to target: {distance:.2f} meters", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)

            else:
                detected_shape = "unknown"
                num_vertices = 0
                vertices = []


            cv2.putText(frame, f"Shape: {detected_shape}", (500, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
            cv2.putText(frame, f"Target Color: {target_color}", (500, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            
            # Display payload status / payload durumunu göster
            red_status = "DROPPED" if red_payload_dropped else "READY"
            blue_status = "DROPPED" if blue_payload_dropped else "READY"
            red_color = (0, 0, 255) if red_payload_dropped else (0, 255, 0)  # Red if dropped, Green if ready
            blue_color = (255, 0, 0) if blue_payload_dropped else (0, 255, 0)  # Blue if dropped, Green if ready
            
            cv2.putText(frame, f"Red Payload (S1): {red_status}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, red_color, 2)
            cv2.putText(frame, f"Blue Payload (S2): {blue_status}", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, blue_color, 2)
            
            # Mission completion status
            if red_payload_dropped and blue_payload_dropped:
                cv2.putText(frame, "MISSION COMPLETE!", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
                        
            #raspta ekransiz calisirken bu satiri yorum satirina al yoksa error verir
            #frame_resized = cv2.resize(frame, resolution)
            cv2.imshow("GTU KUZGUN", frame)
            #cv2.imshow("GTU KUZGUN", mask)


            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
            elif cv2.waitKey(10) & 0xFF == ord('r'):  # Press 'r' to reset servos / close the servos
                servo_controller.reset_servos()
                red_payload_dropped = False
                blue_payload_dropped = False
                print("Servos reset - ready for next mission")
            elif cv2.waitKey(10) & 0xFF == ord('o'):  # Press 'o' to open servos / to place the payloads
                servo_controller.drop_both_payloads() # using the same function that is used to drop as it opens the servos
                print("Servos are open - load the payloads")
            elif cv2.waitKey(10) & 0xFF == ord('t'):  # Press 't' to test servos
                servo_controller.test_servos()
            elif cv2.waitKey(10) & 0xFF == ord('1'):  # Press '1' to manually drop red payload
                if not red_payload_dropped:
                    servo_controller.drop_payload_1()
                    red_payload_dropped = True
                    print("Manual drop: Red payload (Servo 1)")
            elif cv2.waitKey(10) & 0xFF == ord('2'):  # Press '2' to manually drop blue payload
                if not blue_payload_dropped:
                    servo_controller.drop_payload_2()
                    blue_payload_dropped = True
                    print("Manual drop: Blue payload (Servo 2)")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        servo_controller.cleanup()
        camera.release()
    

# Drone bilgilerini al
def get_drone_info():
    vehicle = Vehicle()
    
    velocity = Vehicle.get_speed(vehicle) # 20 # m/s
    altitude = Vehicle.get_altitude(vehicle)# camera_height # metre
    return velocity, altitude

# D???? noktas?n? hesapla
def calculate_drop_point(aircraft_position, velocity, altitude):
    time_to_fall = (2 * altitude / g) ** 0.5
    drop_distance = velocity * time_to_fall

    # burası düzenlenecek !!!!!!!!!!
    if velocity == 0:
        velocity = 2
    if altitude == 0:
        altitude = 2
    
    
    drop_x = int(aircraft_position[0] + drop_distance)
    drop_y = int(aircraft_position[1])
    return (drop_x, drop_y)

# Yükün şu an bırakılması durumunda düşeceği konumun hedefe uzaklığını hesaplama
def calculate_distance(estimated_drop_point, target_position=None):

    if target_position is not None:

        # Yükün şu an bırakılması durumunda düşeceği konumun hedefe uzaklığını hesaplama (x) (piksel)
        horizontal_distance = abs(target_position[0] - estimated_drop_point[0])
    
        # Gerçek mesafeyi hesaplama (x) (metre)
        real_distance_x = horizontal_distance * (max_distance / image_width)

        # Yükün şu an bırakılması durumunda düşeceği konumun hedefe uzaklığını hesaplama (y) (piksel)
        vertical_distance = abs(target_position[1] - estimated_drop_point[1])
    
        # Gerçek mesafeyi hesaplama (y) (metre)
        real_distance_y = vertical_distance * (max_width / image_height)
    
        # hipotenüs formülü ile asıl mesafe hesaplaması
        real_distance = (real_distance_x ** 2 + real_distance_y ** 2) ** 0.5

        return real_distance
    
    return None

# Düşüş noktasını göster
def show_estimated_drop_point(frame, drop_x, drop_y):
    cv2.circle(frame, (drop_x, drop_y), 5, (0, 0, 255), -1) # drop point noktasını çiz
    cv2.line(frame, (drop_x, 0), (drop_x, image_height), (0, 0, 255), 1) # düşüş noktasından geçen y doğrusunu çiz
    cv2.putText(frame, "Drop Point", (drop_x + 10, drop_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA) # drop point metnini yaz
    

# Uçuş verilerini ekrana yazdır
def display_flight_info(frame, altitude, velocity):
    flight_info = f"Altitude: {altitude} m | Velocity: {velocity} m/s"
    cv2.putText(frame, flight_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2, cv2.LINE_AA)



if __name__ == "__main__":
    main()