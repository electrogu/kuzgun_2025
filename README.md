# Kuzgun 2025 Payload Drop System

A computer vision-based payload drop system for autonomous drones with servo-controlled release mechanism.

## Features

- **Computer Vision**: Real-time shape detection and target identification
- **Color-Based Targeting**: Distinguishes between red and blue targets
- **Smart Payload Assignment**: Red payload drops to blue targets, blue payload drops to red targets
- **Dual Servo Control**: Independent control of two payload release mechanisms
- **Drop Point Calculation**: Physics-based drop point estimation considering altitude and velocity
- **Target Validation**: Square shape detection with area and aspect ratio validation
- **Distance Calculation**: Real-world distance measurement using camera parameters
- **Mission Tracking**: Tracks which targets have been hit and payloads dropped

## Hardware Requirements

- Raspberry Pi 4 (recommended) or compatible SBC
- Camera module (USB or RPi Camera)
- 2x Servo motors (SG90 or similar)
- Servo power supply (5V)
- Connecting wires
- Payload release mechanism

## Software Dependencies

- Python 3.7+
- OpenCV
- NumPy
- RPi.GPIO
- DroneKit (for vehicle communication)

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd kuzgun_2025
   ```

2. **Run the setup script** (on Raspberry Pi):
   ```bash
   chmod +x setup_rpi.sh
   ./setup_rpi.sh
   ```

3. **Manual installation** (if needed):
   ```bash
   pip3 install -r requirements.txt
   ```

## Hardware Setup

### Servo Connections
- **Servo 1**: GPIO Pin 18 (Signal), 5V (Power), GND (Ground)
- **Servo 2**: GPIO Pin 19 (Signal), 5V (Power), GND (Ground)

### Camera Connection
- USB Camera: Connect to any available USB port
- RPi Camera: Connect to camera connector on Raspberry Pi

## Configuration

### Servo Settings
Edit `servo_config.ini` to customize:
- GPIO pin assignments
- Servo angles (open/closed positions)
- PWM frequency and timing
- Drop condition parameters

### Camera Parameters
Modify these variables in `main.py`:
- `resolution`: Camera resolution (default: 1280x720)
- `camera_index`: Camera device index (default: 0)
- `FOV_X`, `FOV_Y`: Field of view angles
- `camera_height`: Camera height above ground
- Sensor specifications for GSD calculation

### Target Detection
Adjust color ranges for target detection:
- `lower_red1`, `upper_red1`: Red color range 1
- `lower_red2`, `upper_red2`: Red color range 2  
- `lower_blue`, `upper_blue`: Blue color range

## Usage

### Basic Operation
```bash
python3 main.py
```

### Testing Servos
```bash
python3 test_servos.py
```

### Interactive Controls
During operation:
- **Q**: Quit application
- **R**: Reset both servos to closed position and clear mission status
- **T**: Test servo movement
- **1**: Manually drop red payload (Servo 1)
- **2**: Manually drop blue payload (Servo 2)

## System Operation

1. **Target Detection**: System detects colored shapes in camera feed
2. **Color Identification**: Distinguishes between red and blue targets
3. **Shape Validation**: Validates targets as squares using:
   - Vertex count (must be 4)
   - Aspect ratio (0.95-1.05)
   - Area constraints (configurable)
4. **Drop Point Calculation**: Estimates optimal release point based on:
   - Current altitude
   - Vehicle velocity
   - Physics simulation
5. **Smart Release Logic**: 
   - **Blue Target Detected** → Drops RED payload (Servo 1)
   - **Red Target Detected** → Drops BLUE payload (Servo 2)
   - Each payload drops only once per mission
6. **Mission Completion**: Tracks progress until both payloads are deployed

## File Structure

```
kuzgun_2025/
├── main.py                 # Main application
├── servo_controller.py     # Servo control module
├── camera_handler.py       # Camera interface
├── image_processor.py      # Image processing and detection
├── vehicle.py             # Drone communication
├── XBeeTransmitter.py     # XBee communication
├── test_servos.py         # Servo testing utility
├── setup_rpi.sh          # Raspberry Pi setup script
├── servo_config.ini      # Configuration file
├── requirements.txt      # Python dependencies
└── images/               # Test images
    ├── hexagon.jpg
    ├── square.jpg
    └── triangle.jpg
```

## Technical Details

### Drop Point Physics
The system calculates drop points using:
```python
time_to_fall = sqrt(2 * altitude / gravity)
drop_distance = velocity * time_to_fall
```

### GSD Calculation
Ground Sample Distance for area measurement:
```python
gsd_x = (altitude * sensor_width) / (image_width * focal_length)
gsd_y = (altitude * sensor_height) / (image_height * focal_length)
real_area = (gsd_x * gsd_y) * pixel_area
```

### Servo Control
- **PWM Frequency**: 50Hz (standard for servos)
- **Duty Cycle Range**: 2.5-12.5% (corresponding to 0-180°)
- **Response Time**: ~100ms per movement

## Troubleshooting

### Common Issues

1. **Servo not moving**:
   - Check power supply (5V, adequate current)
   - Verify GPIO connections
   - Test with `test_servos.py`

2. **Camera not detected**:
   - Check USB connection
   - Verify camera index in configuration
   - Test with `lsusb` or `v4l2-ctl --list-devices`

3. **No shape detection**:
   - Adjust color ranges for lighting conditions
   - Check camera focus and resolution
   - Verify target colors match detection ranges

4. **Inaccurate drop calculations**:
   - Calibrate camera parameters
   - Verify altitude and velocity readings
   - Check field of view settings

### Debug Mode
Enable verbose logging by modifying print statements in `main.py`.

## Safety Considerations

- Always test servos before flight
- Ensure adequate power supply for servos
- Verify payload attachment mechanism
- Test drop accuracy in controlled environment
- Follow local aviation regulations

## Contributing

1. Fork the repository
2. Create feature branch
3. Commit changes
4. Submit pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Check troubleshooting section
- Review configuration files
- Test individual components
- Contact development team
