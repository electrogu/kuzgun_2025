from dronekit import connect

# ardupilot (cube orange) ile iletişim kurara ve ihanın temel verilerini okur
# gerçek zmaanlı uçuş verileri
class Vehicle:
    # ardupilot bağlantısını kurar
    def __init__(self, connection_string="/dev/ttyACM0", baud=57600):
        self.vehicle = connect(connection_string, baud=baud, wait_ready=True)
        self.vehicle.wait_ready('autopilot_version')
        print(f"Connected to vehicle on: {connection_string}")

    #irtifa bilgileri 
    def get_altitude(self): #sanirim metre cinsinden
        return self.vehicle.location.global_relative_frame.alt
    
    def get_speed(self): #m/s
        return self.vehicle.groundspeed
    
    def get_gps(self): #m/s
        return self.vehicle.location.global_frame