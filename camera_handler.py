import cv2
from picamera2 import Picamera2
import time

# farklı kamera kaynaklarını tek bir arayüzle yönetmemizi sağlıyor
class CameraHandler:
    # kamera kaynaklarını başlatır
    def __init__(self, resolution=(1280, 720), image_path=None, use_picamera2=True, camera_index=0):
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
            time.sleep(1)  # Kamera a��lmas� i�in bekle

        else:
            self.cap = cv2.VideoCapture(self.camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])


    # aktif olan kamera kaynağından frame alır
    def get_frame(self):
        if self.image_path:
            self.frame = cv2.imread(self.image_path)
            if self.frame is None:
                raise Exception(f"Failed to load image from {self.image_path}")
            self.frame = cv2.resize(self.frame, self.resolution)
            #self.frame = cv2.flip(self.frame, 1)  # Yatayda çevir
            return self.frame

        elif self.use_picamera2:
            frame = self.picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame_bgr

        else:
            ret, frame = self.cap.read()
            if not ret:
                raise Exception("Failed to capture frame from camera.")
            return frame

    # kamera kaynaklarını kapatıyoruz
    def release(self):
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.stop()
        cv2.destroyAllWindows()
