import cv2
import numpy as np

class ImageProcessor:
    def __init__(self, lower_bound, upper_bound):
        self.lower_bound = np.array(lower_bound)
        self.upper_bound = np.array(upper_bound)

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_bound, self.upper_bound)
        return mask

    def find_largest_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            center = self.__get_center_of_contour(largest_contour)
        
            return largest_contour, area, center
        return None, 0, (0, 0)

    def __get_center_of_contour(self, contour):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            center = (0, 0)
        return center