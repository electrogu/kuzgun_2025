from camera_handler import CameraHandler
from image_processor import ImageProcessor
import cv2
import math
import numpy as np
# Define parameters
#            H    S    V
lower_red = [0, 120, 70]
upper_red = [10, 255, 255]
# lower_red = [161, 155, 84]
# upper_red = [179, 255, 255]
lower_blue = [90, 50, 70]
upper_blue = [128, 255, 255]
resolution = (1280, 720)
camera_index = 0
g = 9.80665
image_width = resolution[0] #640  # Kamera çözünürlüğü (piksel cinsinden)
image_height = resolution[1] #480
FOV_Y = 41  # Horizontal field of view (degrees)
FOV_X = 66  # Horizontal field of view (degrees)
camera_height = 10 # altitude  # Kamera yüksekliği (metre cinsinden)
max_distance = camera_height * math.tan(math.radians(FOV_X))  # Kameranın görebileceği maksimum mesafe (metre cinsinden)
max_width = 2 * camera_height * math.tan(math.radians(FOV_Y/2))  # Kameranın görebileceği maksimum geni?lik (metre cinsinden)
aircraft_position = (0, image_height // 2)
sensor_width = 0.00645 # m
sensor_height = 0.00363 # m
focal_length = 0.00474 # m
# (x) gsd = (sensor_width * altitude) / (image_width * focal_length)
# (y) gsd = (sensor_height * altitude) / (image_height * focal_length)
# real_area = (gsd_x * gsd_y) * contour_area

def main():
    # Initialize components
    camera = CameraHandler(camera_index=camera_index, resolution=resolution)
    processor = ImageProcessor([(lower_red, upper_red), (lower_blue, upper_blue)]) # , (lower_blue, upper_blue)

    try:
        while True:

            frame = camera.get_frame()
            mask = processor.process_frame(frame)
            largest_contour, area, center = processor.find_largest_contour(mask)
            
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
                
                """"
                # Angle check for square
                def angle(pt1, pt2, pt3):
                    v1 = pt1 - pt2
                    v2 = pt3 - pt2
                    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                    return np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
    
                is_square_angle = False
                
                if num_vertices == 4 and len(vertices) == 4:
                    pts = [np.array(vertex) for vertex in vertices]
                    angles = []
                    for i in range(4):
                        pt1 = pts[i-1]
                        pt2 = pts[i]
                        pt3 = pts[(i+1)%4]
                        angles.append(angle(pt1, pt2, pt3))
                        print(angle(pt1, pt2, pt3))
                        cv2.putText(frame, f"Angles: {angle(pt1, pt2, pt3)}", (1000, 30+i*100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
                    is_square_angle = all(80 <= a <= 100 for a in angles)  # Allowing ?10? tolerance
                """
                control = aspect_ratio >= 0.95 and aspect_ratio <= 1.05
                drop = estimated_drop_point
                print(estimated_drop_point, center, area, real_area, 5 >= real_area >= 3, control, num_vertices) # s?ras?yla ?u de?erlerin ??kt?s? (tahmini d??me noktas?, hedefin merkezi, pixel alan, ger?ek alan, ger?ek alan 3'ten b?y?k m??, kare mi?)
                common_control = (drop[0] - 15, drop[1] - 15) <= center <= (drop[0] + 15, drop[1] + 15) and control and num_vertices == 4
                if (common_control and (4.5 >= real_area >= 3.5 or 16.5 >= real_area >= 15.5)):
                    print("yuk birakildi")
                    pass
                
                if ( (5 >= real_area >= 3 or 17 >= real_area >= 15) and num_vertices == 4): # 5 > real_area and is_square_angle
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, center, 5, (0, 255, 255), -1)
                    # Draw the vertices on the frame
                    for vertex in vertices:
                        cv2.circle(frame, tuple(vertex), 5, (255, 0, 0), -1)
                

                # Düşüş noktası ve hedef arasındaki mesafeyi hesapla
                distance = calculate_distance(estimated_drop_point, center)

                # Düşüş noktası ve hedef arasındaki mesafeyi ekrana yazd?r
                cv2.putText(frame, f"Distance to target: {distance:.2f} meters", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)

            else:
                detected_shape = "unknown"
                num_vertices = 0
                vertices = []


            cv2.putText(frame, f"Shape: {detected_shape}", (500, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
            #raspta ekransiz calisirken bu satiri yorum satirina al yoksa error verir
            cv2.imshow("GTU KUZGUN", frame)
            cv2.imshow("GTU KUZGUN", mask)
            

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error: {e}")

    finally:
        camera.release()
    

# Drone bilgilerini al
def get_drone_info():
    velocity = 20 # m/s
    altitude = camera_height  # metre
    return velocity, altitude

# D???? noktas?n? hesapla
def calculate_drop_point(aircraft_position, velocity, altitude):
    time_to_fall = (2 * altitude / g) ** 0.5
    drop_distance = velocity * time_to_fall
    drop_x = int(aircraft_position[0] + drop_distance)
    drop_y = int(aircraft_position[1])
    return (drop_x, drop_y)

# Y?k?n ?u an b?rak?lmas? durumunda d??ece?i konumun hedefe uzakl???n? hesaplama
def calculate_distance(estimated_drop_point, target_position=None):

    if target_position is not None:

        # Y?k?n ?u an b?rak?lmas? durumunda d??ece?i konumun hedefe uzakl???n? hesaplama (x)
        horizontal_distance = abs(target_position[0] - estimated_drop_point[0])
    
        # Ger?ek mesafeyi hesaplama (x)
        real_distance_x = horizontal_distance * (max_distance / image_width)

        # Y?k?n ?u an b?rak?lmas? durumunda d??ece?i konumun hedefe uzakl???n? hesaplama (y)
        vertical_distance = abs(target_position[1] - estimated_drop_point[1])
    
        # Ger?ek mesafeyi hesaplama (y)
        real_distance_y = vertical_distance * (max_width / image_height)
    
        real_distance = (real_distance_x ** 2 + real_distance_y ** 2) ** 0.5

        return real_distance
    
    return None

# D???? noktas?n? g?ster
def show_estimated_drop_point(frame, drop_x, drop_y):
    cv2.circle(frame, (drop_x, drop_y), 5, (0, 0, 255), -1)
    cv2.line(frame, (drop_x, 0), (drop_x, image_height), (0, 0, 255), 1)
    cv2.putText(frame, "Drop Point", (drop_x + 10, drop_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

# U?u? verilerini ekrana yazd?r
def display_flight_info(frame, altitude, velocity):
    flight_info = f"Altitude: {altitude} m | Velocity: {velocity} m/s"
    cv2.putText(frame, flight_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2, cv2.LINE_AA)



if __name__ == "__main__":
    main()