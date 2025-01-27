import cv2

class CameraHandler:
    # resolution kendi ekraninizin cozunurlugune gore degistirilebilir
    # kamera cozuunurlugu dikkate alinabilir
    def __init__(self, camera_index=0, resolution=(1280, 720)):
        self.camera_index = camera_index
        self.resolution = resolution
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture frame from camera.")
        return frame

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()




'''

from picamera2 import Picamera2
import numpy as np

class CameraHandler:
    def __init__(self, camera_index=0, resolution=(1280, 720)):
        self.camera_index = camera_index
        self.resolution = resolution
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": resolution}))
        self.picam2.start()

    def get_frame(self):
        # Capture a frame from the camera using picamera2
        frame = self.picam2.capture_array()
        # Convert the frame from RGB (default) to BGR (OpenCV format)
        frame_bgr = frame[..., ::-1]
        return frame_bgr

    def release(self):
        self.picam2.stop()

'''