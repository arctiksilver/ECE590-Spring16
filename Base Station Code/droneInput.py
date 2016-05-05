
class droneInput:
    def __init__(self, data = None):
        if data is None:
            self.throttle = 0.0
            self.yaw = 0.0
            self.pitch = 0.0
            self.role = 0.0
        else:
            self.throttle = data[0]
            self.yaw = data[1]
            self.pitch = data[2]
            self.role = data[3]
