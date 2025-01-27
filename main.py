from camera_handler import CameraHandler
from image_processor import ImageProcessor
#from control_unit import ControlUnit
import cv2

if __name__ == "__main__":
    # Define parameters
    #            H    S    V
    lower_red = [161, 155, 84]
    upper_red = [179, 255, 255]
    resolution = (1280, 720)
    camera_index = 0

    # Initialize components
    camera = CameraHandler(camera_index=camera_index ,resolution=resolution)
    processor = ImageProcessor(lower_red, upper_red)
    #control = ControlUnit()

    try:
        while True:
            frame = camera.get_frame()
            mask = processor.process_frame(frame)
            largest_contour, area, center = processor.find_largest_contour(mask)




            #control.check_and_trigger(largest_contour)
            
            
            

            # Display the results
                
            if largest_contour is not None:
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)



            cv2.imshow("GTU KUZGUN", frame)

            #cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error: {e}")

    finally:
        camera.release()
