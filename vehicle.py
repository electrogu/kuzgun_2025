from dronekit import connect,Vehicle,LocationGlobalRelative

class Vehicle:
    def __init__(self, connection_string="/dev/ttyUSB0", baud=57600):
        self.vehicle = connect(connection_string, baud=baud, wait_ready=True)
        self.vehicle.wait_ready('autopilot_version')
        print(f"Connected to vehicle on: {connection_string}")

    def get_altitude(self): #sanirim metre cinsinden
        return self.vehicle.location.global_relative_frame.alt
    
    def get_speed(self): #m/s
        return self.vehicle.groundspeed