import cv2
import serial
import numpy as np

class XBeeTransmitter:
    def __init__(self, port, baudrate=9600): # xbee modülüne seri port bağlantısı kurar
        self.ser = serial.Serial(port, baudrate)
    
    def send_frame(self, frame):
        # Encode frame as JPEG
        _, buffer = cv2.imencode('.jpg', frame) # OpenCV frame'ini JPEG formatına sıkıştırır (boyut küçültme için)
        
        # Convert to bytes
        frame_bytes = buffer.tobytes() # JPEG verisini byte dizisine çevir
        
        # Send the length of the frame first
        # Alıcının ne kadar veri bekleyeceğini bilmesi için frame boyutunu 4 byte olarak gönderir
        self.ser.write(len(frame_bytes).to_bytes(4, byteorder='big'))
        
        # Send the frame bytes
        # Asıl görüntü verisini gönderir
        self.ser.write(frame_bytes)
    
    def close(self): # port bağlantsını sonlandırır
        self.ser.close()