from camera_handler import CameraHandler
from image_processor import ImageProcessor
import cv2
import time

if __name__ == "__main__":
    # Define parameters
    #            H    S    V
    lower_red = [161, 155, 84]
    upper_red = [179, 255, 255]
    lower_blue = [94, 80, 2]
    upper_blue = [126, 255, 255]
    resolution = (1280, 720)
    camera_index = 0
    g = 9.80665

    # Initialize components
    camera = CameraHandler(camera_index=camera_index, resolution=resolution)
    processor = ImageProcessor([(lower_red, upper_red), (lower_blue, upper_blue)])

    try:
        while True:


            frame = camera.get_frame()
            mask = processor.process_frame(frame)
            largest_contour, area, center = processor.find_largest_contour(mask)

            if largest_contour is not None:
                detected_shape, num_vertices, vertices = processor.detect_shape(largest_contour)
                
                velocity = 25 # m/s
                altitude = 20 # m

                # calcualte fall time
                fall_time = ((2 * altitude) / g) ** 0.5 

                fall_distance = velocity * fall_time

                ##hesaplamalar

                impact_coordinates = (360,540)

                if center == impact_coordinates:
                    #ates edildi
                    pass
                
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
                cv2.circle(frame, center, 5, (0, 255, 255), -1)

                # Draw the vertices on the frame
                for vertex in vertices:
                    cv2.circle(frame, tuple(vertex), 5, (255, 0, 0), -1)
            else:
                detected_shape = "unknown"
                num_vertices = 0
                vertices = []


            cv2.putText(frame, f"Shape: {detected_shape}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            #raspta ekransiz calisirken bu satiri yorum satirina al yoksa error verir
            cv2.imshow("GTU KUZGUN", frame)
            

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

            



    except Exception as e:
        print(f"Error: {e}")

    finally:
        camera.release()