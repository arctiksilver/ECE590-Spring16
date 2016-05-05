# Drone Flight controller
# ECE590 - Robot Design & Implimentation - Spring 2016
# Chris Glomb

# This code must be run with root priviliges

import sys
import time
import PID
from droneData import *
from Adafruit_BNO055 import BNO055 # Sensor libraries provided by Adafruit Industries Copyright (c) 2015
import Adafruit_PCA9685
import socket
import struct
import fcntl

# Create and configure the BNO055 sensor connection.
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

# Initialise the PWM driver using the default I2C address
pwm = Adafruit_PCA9685.PCA9685()

# rate and stab PID controllers for role, pitch, yaw

pids = [PID.PID() for i in range(6)]

pids[PID_PITCH_RATE].setKp(0.5)
#pids[PID_PITCH_RATE].setKd(0.1)
#pids[PID_PITCH_RATE].setKi(1)

pids[PID_ROLL_RATE].setKp(0.5)
#pids[PID_ROLL_RATE].setKd(0.1)
#pids[PID_ROLL_RATE].setKi(1)

pids[PID_YAW_RATE].setKp(2.5)
#pids[PID_YAW_RATE].setKi(1)

pids[PID_PITCH_STAB].setKp(7.5)
pids[PID_ROLL_STAB].setKp(7.5)
pids[PID_YAW_STAB].setKp(10)


# Configure PWM output frequency
pwm.set_pwm_freq(450)

# set output to ESC minimum throttle
pwm.set_pwm(motor_FL, 0, 2000)
pwm.set_pwm(motor_FR, 0, 2000)
pwm.set_pwm(motor_BL, 0, 2000)
pwm.set_pwm(motor_BR, 0, 2000)
time.sleep(0.01)
pwm.set_pwm(motor_FL, 0, 3300)
pwm.set_pwm(motor_FR, 0, 3300)
pwm.set_pwm(motor_BL, 0, 3300)
pwm.set_pwm(motor_BR, 0, 3300)
time.sleep(0.01)
pwm.set_pwm(motor_FL, 0, 2000)
pwm.set_pwm(motor_FR, 0, 2000)
pwm.set_pwm(motor_BL, 0, 2000)
pwm.set_pwm(motor_BR, 0, 2000)

# configure socket connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
my_ip = socket.inet_ntoa(fcntl.ioctl(sock.fileno(),0x8915, struct.pack('256s', 'wlan0'[:15]))[20:24]) # IP of wlan0
port = 5005
addr = (my_ip, port)
sock.bind(addr)
sock.setblocking(0)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print BNO055 system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))

def calibrate():
    cal = False
    i = 0
    while cal == False:
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        pwm.set_pwm(SYS_CAL_LED,0, 1365*sys)
        pwm.set_pwm(GYRO_CAL_LED,0, 1365*gyro)
        pwm.set_pwm(ACCEL_CAL_LED,0, 1365*accel)
        pwm.set_pwm(MAG_CAL_LED,0, 1365*mag)
        print sys, gyro, accel, mag
        time.sleep(0.1)
        if(sys==gyro==accel==mag==3):
            i += 1
            if i == 10:
                cal = True
# End calibrate()

def wrap_180(x):
    if x < -180:
        x += 360
    elif x > 180:
        x -= 360
    return x

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
def clamp(n, minn, maxn): 
    return min(max(n, minn), maxn)

def setThrottle(throttle):
    return int(map(throttle, 0, 1000, 2000, 3200))

def setMotors(throttle):
    pwm.set_pwm(motor_FL, 0, throttle)
    pwm.set_pwm(motor_FR, 0, throttle)
    pwm.set_pwm(motor_BL, 0, throttle)
    pwm.set_pwm(motor_BR, 0, throttle)    

def flightControl():
    desired = droneInput()
    yaw_target = 0.0
    count = 0
    flying = False
    pidGain = 15
    userGain = 10
    stabClampRange = 250
    rateClampRange = 500
    
    while True:
        #count += 1
        try:
            # get user input
            data, addr = sock.recvfrom(1024)
            rc = droneInput(struct.unpack('ffff', data))
            desired.throttle += rc.throttle*50
            if desired.throttle > 1000:
                desired.throttle = 1000
            elif desired.throttle < 0:
                desired.throttle = 0

            desired.yaw = map(rc.yaw, -0.5, 0.5, -50, 50)
            desired.pitch = map(rc.pitch, -0.5, 0.5, -15, 15)
            desired.roll = -map(rc.roll, -0.5, 0.5, -15, 15)

        except socket.error:
            pass
        
        
        imu = euler(bno.read_euler())
        imu.heading = imu.heading - 180
        gyro = axis(bno.read_gyroscope())
        
        
        if desired.throttle > 100:
            if flying == False:
                for pid in pids:
                    pid.clear()
                pids[PID_YAW_RATE].setWindup(50)
                pids[PID_PITCH_RATE].setWindup(50)
                pids[PID_ROLL_RATE].setWindup(50)
                flying = True
                imu = euler(bno.read_euler())
                desired.yaw = imu.heading
                yaw_target = desired.yaw
            
            yaw_stab_output = clamp(pids[PID_YAW_STAB].update(wrap_180(yaw_target - imu.heading)), -stabClampRange, stabClampRange)
            pitch_stab_output = clamp(pids[PID_PITCH_STAB].update(desired.pitch - imu.pitch), -stabClampRange, stabClampRange)
            roll_stab_output = clamp(pids[PID_ROLL_STAB].update(desired.roll - imu.roll), -stabClampRange, stabClampRange)
            
            if abs(desired.yaw) > 5:
                yaw_stab_output = desired.yaw
                yaw_target = imu.heading
            
            yaw_output = clamp(pids[PID_YAW_RATE].update(yaw_stab_output - gyro.yaw), -rateClampRange, rateClampRange)
            pitch_output = clamp(pids[PID_PITCH_RATE].update(pitch_stab_output - gyro.pitch), -rateClampRange, rateClampRange)
            roll_output = clamp(pids[PID_ROLL_RATE].update(roll_stab_output - gyro.roll), -rateClampRange, rateClampRange)
            
            #yaw_output = 0
            
            # command motors
            throttle_output = map(desired.throttle, 0, 1000, 2000, 3200)
            
            FL_out = int(throttle_output - roll_output - pitch_output - yaw_output)
            BL_out = int(throttle_output - roll_output + pitch_output + yaw_output)
            FR_out = int(throttle_output + roll_output - pitch_output + yaw_output)
            BR_out = int(throttle_output + roll_output + pitch_output - yaw_output)
            
            pwm.set_pwm(motor_FL, 0, FL_out)
            pwm.set_pwm(motor_BL, 0, BL_out)
            pwm.set_pwm(motor_FR, 0, FR_out)
            pwm.set_pwm(motor_BR, 0, BR_out)
            
            #if count > 15:
             #   print throttle_output, roll_output, pitch_output
             #   print 'IMU', imu.heading, imu.pitch, imu.roll
             #   print 'PID out', yaw_output, pitch_output, roll_output
             #   print 'FL =', FL_out, 'BL =', BL_out, 'FR =', FR_out, 'BR =', BR_out
             #   count = 0
        else:
            flying = False
            pwm.set_pwm(motor_FL, 0, 2000)
            pwm.set_pwm(motor_FR, 0, 2000)
            pwm.set_pwm(motor_BL, 0, 2000)
            pwm.set_pwm(motor_BR, 0, 2000)
            
        time.sleep(0.01)
    


# calibrate BNO055 sensor
calibrate()
# start the controller  
flightControl()







