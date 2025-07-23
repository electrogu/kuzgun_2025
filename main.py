#!/usr/bin/env python3

from camera_handler import CameraHandler
from image_processor import ImageProcessor
from vehicle import Vehicle
from servo_controller import ServoController
import cv2
import math
import numpy as np

# HSV Color ranges
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([130, 255, 255])

# Camera and flight parameters
resolution = (1280, 720)
camera_index = 0
g = 9.80665
image_width, image_height = resolution
FOV_Y = 41  # Vertical field of view (degrees)
FOV_X = 66  # Horizontal field of view (degrees)
camera_height = 10  # Initial altitude (meters)
rotated_degree = 15

# Calculate max distances
max_distance_back = camera_height * math.tan(math.radians(rotated_degree))
max_distance_front = camera_height * math.tan(math.radians(FOV_X - rotated_degree))
max_distance = max_distance_front + max_distance_back
max_width = 2 * camera_height * math.tan(math.radians(FOV_Y/2))
aircraft_position = ((image_width*(max_distance_back/(max_distance_back+max_distance_front))), image_height // 2)

# Camera sensor specifications
sensor_width = 0.00645  # m
sensor_height = 0.00363  # m
focal_length = 0.00474  # m

def main():
    # Initialize components
    vehicle = Vehicle()
    camera = CameraHandler(camera_index=camera_index, resolution=resolution)
    processor = ImageProcessor([(lower_red1, upper_red1), (lower_red2, upper_red2), (lower_blue, upper_blue)])
    servo_controller = ServoController(servo1_pin=18, servo2_pin=19)
    
    # Payload status
    red_payload_dropped = False
    blue_payload_dropped = False

    cv2.namedWindow("GTU KUZGUN", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("GTU KUZGUN", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:
            frame = camera.get_frame()
            if frame is None:
                print("Failed to get frame from camera")
                break
                
            mask = processor.process_frame(frame)
            largest_contour, area, center = processor.find_largest_contour(mask)
            
            # Color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Determine target color
            target_color = "unknown"
            if largest_contour is not None:
                contour_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                cv2.drawContours(contour_mask, [largest_contour], -1, 255, -1)
                
                red_pixels = cv2.countNonZero(cv2.bitwise_and(contour_mask, red_mask))
                blue_pixels = cv2.countNonZero(cv2.bitwise_and(contour_mask, blue_mask))
                
                if red_pixels > blue_pixels and red_pixels > 100:
                    target_color = "red"
                elif blue_pixels > red_pixels and blue_pixels > 100:
                    target_color = "blue"
            
            # Get drone info and calculate GSD
            velocity, altitude = get_drone_info(vehicle)
            if altitude <= 0:
                altitude = 10  # Default safe altitude
                
            gsd_x = (altitude * sensor_width) / (image_width * focal_length)
            gsd_y = (altitude * sensor_height) / (image_height * focal_length)
            real_area = (gsd_x * gsd_y) * area if area > 0 else 0
            
            # Display flight info
            display_flight_info(frame, altitude, velocity)
            estimated_drop_point = calculate_drop_point(aircraft_position, velocity, altitude)
            show_estimated_drop_point(frame, estimated_drop_point[0], estimated_drop_point[1])

            if largest_contour is not None:
                # Shape detection
                detected_shape, num_vertices, vertices = processor.detect_shape(largest_contour)
                x, y, w, h = cv2.boundingRect(largest_contour)
                aspect_ratio = float(w)/h
                
                # Position and shape controls
                control = 0.9 <= aspect_ratio <= 1.1
                dx, dy = estimated_drop_point
                cx, cy = center
                common_control = (dx - 15) <= cx <= (dx + 15) and control and num_vertices == 4
                
                # Debug output
                print(f"Target: {target_color}, Area: {real_area:.2f}, Vertices: {num_vertices}, Center: {center}, Drop Point: {estimated_drop_point}")
                
                # Draw contours based on target type and area
                if target_color == "red" and (4.5 >= real_area >= 3.5):
                    cv2.drawContours(frame, [largest_contour], -1, (0, 0, 255), 3)
                    cv2.putText(frame, "RED TARGET", (center[0]-50, center[1]-30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    if common_control and not blue_payload_dropped:
                        print("Red target detected - dropping BLUE payload")
                        servo_controller.drop_payload_2()
                        blue_payload_dropped = True
                        
                elif target_color == "blue" and (16.5 >= real_area >= 14.5):
                    cv2.drawContours(frame, [largest_contour], -1, (255, 0, 0), 3)
                    cv2.putText(frame, "BLUE TARGET", (center[0]-50, center[1]-30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    if common_control and not red_payload_dropped:
                        print("Blue target detected - dropping RED payload")
                        servo_controller.drop_payload_1()
                        red_payload_dropped = True
                
                # Always draw center and vertices for detected contours
                cv2.circle(frame, center, 5, (0, 255, 255), -1)
                for vertex in vertices:
                    cv2.circle(frame, tuple(vertex), 5, (255, 255, 255), -1)
                
                # Display distance to target
                distance = calculate_distance(estimated_drop_point, center)
                cv2.putText(frame, f"Distance: {distance:.2f}m", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)

            # Display status information
            cv2.putText(frame, f"Shape: {detected_shape}", (500, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
            cv2.putText(frame, f"Target: {target_color}", (500, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            
            # Payload status
            red_status = "DROPPED" if red_payload_dropped else "READY"
            blue_status = "DROPPED" if blue_payload_dropped else "READY"
            cv2.putText(frame, f"Red Payload: {red_status}", (10, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if red_payload_dropped else (0, 255, 0), 2)
            cv2.putText(frame, f"Blue Payload: {blue_status}", (10, 115), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0) if blue_payload_dropped else (0, 255, 0), 2)
            
            if red_payload_dropped and blue_payload_dropped:
                cv2.putText(frame, "MISSION COMPLETE!", (10, 190), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
            
            # Show frame
            cv2.imshow("GTU KUZGUN", frame)
            
            # Keyboard controls
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                servo_controller.reset_servos()
                red_payload_dropped = blue_payload_dropped = False
                print("Servos reset")
            elif key == ord('o'):
                servo_controller.drop_both_payloads()
                print("Servos opened for loading")
            elif key == ord('1') and not red_payload_dropped:
                servo_controller.drop_payload_1()
                red_payload_dropped = True
                print("Manual red payload drop")
            elif key == ord('2') and not blue_payload_dropped:
                servo_controller.drop_payload_2()
                blue_payload_dropped = True
                print("Manual blue payload drop")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        servo_controller.cleanup()
        camera.release()
        cv2.destroyAllWindows()

def get_drone_info(vehicle_instance):
    try:
        velocity = vehicle_instance.get_speed() or 15  # Default 15 m/s
        altitude = vehicle_instance.get_altitude() or 20  # Default 20 m
        return velocity, altitude
    except Exception as e:
        print(f"Drone info error: {e}")
        return 15, 20  # Fallback values

def calculate_drop_point(aircraft_position, velocity, altitude):
    # burası düzenlenecek !!!!!!!!!!
    if velocity == 0:
        velocity = 20
    if altitude == 0:
        altitude = 20
        
    time_to_fall = abs((2 * altitude / g)) ** 0.5
    drop_distance = velocity * time_to_fall
    
    drop_x = int(aircraft_position[0] + drop_distance)
    drop_y = int(aircraft_position[1])
    return (drop_x, drop_y)

def calculate_distance(point1, point2):
    if point1 is None or point2 is None:
        return 0
    dx = (point1[0] - point2[0]) * (max_distance / image_width)
    dy = (point1[1] - point2[1]) * (max_width / image_height)
    return math.sqrt(dx**2 + dy**2)

def show_estimated_drop_point(frame, x, y):
    cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
    cv2.line(frame, (x, 0), (x, image_height), (0, 0, 255), 1)
    cv2.putText(frame, "Drop Point", (x + 10, y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

def display_flight_info(frame, altitude, velocity):
    info = f"Altitude: {altitude:.1f}m | Velocity: {velocity:.1f}m/s"
    cv2.putText(frame, info, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2, cv2.LINE_AA)

if __name__ == "__main__":
    main()