import cv2
import numpy as np

class ImageProcessor:
    def __init__(self, color_ranges):
        self.color_ranges = [(np.array(lower), np.array(upper)) for lower, upper in color_ranges]

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype="uint8")
        
        for lower, upper in self.color_ranges:
            mask += cv2.inRange(hsv, lower, upper)
        
        return mask

    def find_largest_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            center = self.__get_center_of_contour(largest_contour)

            return largest_contour, area, center
        return None, 0, (0, 0)
    
    def detect_shape(self, contour):
        # Approximate the contour
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Check the number of vertices
        num_vertices = len(approx)
        
        # Determine the shape based on the number of vertices
        if num_vertices == 4:
            shape = "square"
        elif num_vertices == 3:
            shape = "triangle"
        elif num_vertices == 6:
            shape = "hexagon"
        else:
            shape = "unknown"
        
        # Get the coordinates of the vertices
        vertices = approx.reshape(-1, 2)
        
        return shape, num_vertices, vertices

    def __get_center_of_contour(self, contour):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            center = (0, 0)
        return center