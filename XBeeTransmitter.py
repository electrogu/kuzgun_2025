import cv2
import serial
import numpy as np

class XBeeTransmitter:
    def __init__(self, port, baudrate=9600):
        self.ser = serial.Serial(port, baudrate)
    
    def send_frame(self, frame):
        # Encode frame as JPEG
        _, buffer = cv2.imencode('.jpg', frame)
        # Convert to bytes
        frame_bytes = buffer.tobytes()
        # Send the length of the frame first
        self.ser.write(len(frame_bytes).to_bytes(4, byteorder='big'))
        # Send the frame bytes
        self.ser.write(frame_bytes)
    
    def close(self):
        self.ser.close()