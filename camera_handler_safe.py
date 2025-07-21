import cv2
import time
import os

# Picamera2 için güvenli import
try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    print("⚠️  Picamera2 mevcut değil - webcam kullanılacak")
    PICAMERA_AVAILABLE = False

class CameraHandler:
    def __init__(self, resolution=(1280, 720), image_path=None, use_picamera2=True, camera_index=0):
        self.resolution = resolution
        self.image_path = image_path
        self.use_picamera2 = use_picamera2 and PICAMERA_AVAILABLE
        self.camera_index = camera_index
        self.cap = None
        self.picam2 = None
        self.frame = None

        if image_path:
            if os.path.exists(image_path):
                self.frame = cv2.imread(image_path)
                if self.frame is None:
                    raise Exception(f"Failed to load image from {image_path}")
                self.frame = cv2.resize(self.frame, resolution)
                print(f"📷 Test görüntüsü yüklendi: {image_path}")
            else:
                print(f"⚠️  Test görüntüsü bulunamadı: {image_path}")
                print("Webcam kullanılacak...")
                self._init_webcam()

        elif self.use_picamera2:
            try:
                self.picam2 = Picamera2()
                self.picam2.configure(self.picam2.create_preview_configuration(main={"size": resolution}))
                self.picam2.start()
                time.sleep(1)  # Kamera açılması için bekle
                print("📷 Picamera2 başlatıldı")
            except Exception as e:
                print(f"⚠️  Picamera2 başlatılamadı: {e}")
                print("Webcam kullanılacak...")
                self._init_webcam()

        else:
            self._init_webcam()
    
    def _init_webcam(self):
        """Webcam başlatma fonksiyonu"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                raise Exception(f"Webcam açılamadı: index {self.camera_index}")
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.use_picamera2 = False
            self.image_path = None
            print(f"📷 Webcam başlatıldı: index {self.camera_index}")
        except Exception as e:
            print(f"❌ Webcam başlatılamadı: {e}")
            # Son çare olarak test görüntüsü kullan
            test_images = ["images/square.jpg", "images/hexagon.jpg", "images/triangle.jpg"]
            for img_path in test_images:
                if os.path.exists(img_path):
                    self.image_path = img_path
                    self.frame = cv2.imread(img_path)
                    self.frame = cv2.resize(self.frame, self.resolution)
                    print(f"📷 Fallback: {img_path} kullanılıyor")
                    break
            else:
                raise Exception("Hiçbir kamera veya test görüntüsü bulunamadı!")

    def get_frame(self):
        if self.image_path:
            self.frame = cv2.imread(self.image_path)
            if self.frame is None:
                raise Exception(f"Failed to load image from {self.image_path}")
            self.frame = cv2.resize(self.frame, self.resolution)
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

    def release(self):
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.stop()
        cv2.destroyAllWindows()
        print("📷 Kamera kaynakları temizlendi")
