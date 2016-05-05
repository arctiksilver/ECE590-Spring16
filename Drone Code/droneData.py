
# Constants
SYS_CAL_LED = 2
GYRO_CAL_LED = 3
ACCEL_CAL_LED = 12
MAG_CAL_LED = 13

motor_FL = 1
motor_FR = 14
motor_BL = 0
motor_BR = 15

PID_PITCH_RATE = 0
PID_ROLL_RATE = 1
PID_PITCH_STAB = 2
PID_ROLL_STAB = 3
PID_YAW_RATE = 4
PID_YAW_STAB = 5

class droneInput:
    def __init__(self, data = None):
        if data is None:
            self.throttle = 0.0
            self.yaw = 0.0
            self.pitch = 0.0
            self.roll = 0.0
        else:
            self.throttle = data[0]
            self.yaw = data[1]
            self.pitch = data[2]
            self.roll = data[3]

class axis:
    def __init__(self, data = None):
        if data is None:
            self.pitch = 0.0
            self.roll = 0.0
            self.yaw = 0.0
        else:
            self.pitch = data[0]
            self.roll = data[1]
            self.yaw = data[2]
            
class euler:
    def __init__(self, data = None):
        if data is None:
            self.heading = 0.0
            self.roll = 0.0
            self.pitch = 0.0
        else:
            self.heading = data[0]
            self.roll = data[1]
            self.pitch = data[2]