import cv2
from picamera2 import Picamera2
import time

class CameraHandler:
    def __init__(self, resolution=(1280, 720), image_path=None, use_picamera2=False, camera_index=0):
        self.resolution = resolution
        self.image_path = image_path
        self.use_picamera2 = use_picamera2
        self.camera_index = camera_index
        self.cap = None
        self.picam2 = None
        self.frame = None

        if image_path:
            self.frame = cv2.imread(image_path)
            if self.frame is None:
                raise Exception(f"Failed to load image from {image_path}")
            self.frame = cv2.resize(self.frame, resolution)

        elif use_picamera2:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration(main={"size": resolution}))
            self.picam2.start()
            time.sleep(1)  # Kamera açýlmasý için bekle

        else:
            self.cap = cv2.VideoCapture(self.camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])


    def get_frame(self):
        if self.image_path:
            self.frame = cv2.imread(self.image_path)
            if self.frame is None:
                raise Exception(f"Failed to load image from {self.image_path}")
            self.frame = cv2.resize(self.frame, self.resolution)
            return self.frame

        elif self.use_picamera2:
            frame = self.picam2.capture_array()
            cv2.imshow("Raspberry Kamera", frame)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame_bgr

        else:
            ret, frame = self.cap.read()
            if not ret:
                raise Exception("Failed to capture frame from camera.")
            return frame

    def release(self):
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.stop()
        cv2.destroyAllWindows()
