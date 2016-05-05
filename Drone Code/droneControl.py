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

pids[PID_PITCH_RATE].setKp(0.7)
#pids[PID_PITCH_RATE].setKi(1)
pids[PID_PITCH_RATE].setWindup(50)

pids[PID_ROLL_RATE].setKp(0.7)
#pids[PID_ROLL_RATE].setKi(1)
pids[PID_ROLL_RATE].setWindup(50)

pids[PID_YAW_RATE].setKp(2.5)
#pids[PID_YAW_RATE].setKi(1)
pids[PID_YAW_RATE].setWindup(50)

pids[PID_PITCH_STAB].setKp(4.5)
pids[PID_ROLL_STAB].setKp(4.5)
pids[PID_YAW_STAB].setKp(10)

# Variables
Calibrated = False

# Configure PWM output frequency
pwm.set_pwm_freq(400) # Produces measured output frequency ~420Hz

# set output to ESC minimum throttle
pwm.set_pwm(motor_FL, 0, 1900)
pwm.set_pwm(motor_FR, 0, 1900)
pwm.set_pwm(motor_BL, 0, 1900)
pwm.set_pwm(motor_BR, 0, 1900)

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
    while Calibrated == False:
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        pwm.set_pwm(SYS_CAL_LED,0, 1365*sys)
        pwm.set_pwm(GYRO_CAL_LED,0, 1365*gyro)
        pwm.set_pwm(ACCEL_CAL_LED,0, 1365*accel)
        pwm.set_pwm(MAG_CAL_LED,0, 1365*mag)
    
        if(sys==gyro==accel==mag==3):
            Calibrated = True
# End calibrate()

def wrap_180(x):
    if x < -180:
        x += 360
    elif x > 180:
        x -= 360
    return x

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def setThrottle(throttle):
    return int(map(throttle, 0, 1000, 2000, 3200))

def setMotors(throttle):
    pwm.set_pwm(motor_FL, 0, throttle)
    pwm.set_pwm(motor_FR, 0, throttle)
    pwm.set_pwm(motor_BL, 0, throttle)
    pwm.set_pwm(motor_BR, 0, throttle)    

def flightControl():
    current = droneInput()
    count = 0
    flying = False
    pidGain = 10
    
    while True:
        count += 1
        try:
            # get user input
            data, addr = sock.recvfrom(1024)
            user = droneInput(struct.unpack('ffff', data))
            current.throttle += user.throttle*40
            if current.throttle > 1000:
                current.throttle = 1000
            elif current.throttle < 0:
                current.throttle = 0
            current.yaw = user.yaw*20
            current.pitch = user.pitch*20
            current.role = user.role*20
            #current.yaw += user.yaw
            #current.yaw = wrap_180(current.yaw)
            #setMotors(int(map(current.throttle, 0, 1000, 2000, 3200)))
            #print current.throttle
        except socket.error:
            pass
        
        if current.throttle > 50:
            if flying == False:
                for pid in pids:
                    pid.clear()
                    flying = True
            
            # set goal rate into PIDs
            pids[PID_YAW_RATE].SetPoint = current.yaw
            pids[PID_PITCH_RATE].SetPoint = -current.pitch
            pids[PID_ROLL_RATE].SetPoint = current.role
            
            # get current gyro reading
            gyro = axis(bno.read_gyroscope())
            
            # get output from PIDs
            pids[PID_YAW_RATE].update(gyro.z)
            yaw_output = pids[PID_YAW_RATE].output*pidGain
            pids[PID_PITCH_RATE].update(gyro.x)
            pitch_output = pids[PID_PITCH_RATE].output*pidGain
            pids[PID_ROLL_RATE].update(gyro.y)
            roll_output = pids[PID_ROLL_RATE].output*pidGain
            
            # command motors
            throttle_output = map(current.throttle, 0, 1000, 2000, 3200)
            
            FL_out = int(throttle_output + roll_output + pitch_output)
            BL_out = int(throttle_output + roll_output - pitch_output)
            FR_out = int(throttle_output - roll_output + pitch_output)
            BR_out = int(throttle_output - roll_output - pitch_output)
            
            pwm.set_pwm(motor_FL, 0, FL_out)
            pwm.set_pwm(motor_BL, 0, BL_out)
            pwm.set_pwm(motor_FR, 0, FR_out)
            pwm.set_pwm(motor_BR, 0, BR_out)
            
            if count > 15:
                #print throttle_output, roll_output, pitch_output
                print 'FL =', FL_out, 'BL =', BL_out, 'FR =', FR_out, 'BR =', BR_out
                count = 0
        else:
            flying = False
            pwm.set_pwm(motor_FL, 0, 1900)
            pwm.set_pwm(motor_FR, 0, 1900)
            pwm.set_pwm(motor_BL, 0, 1900)
            pwm.set_pwm(motor_BR, 0, 1900)
            
            
            
            # Read the Euler angles for heading, roll, pitch (all in degrees)
            #sensor = euler(bno.read_euler())
            #sensor.heading = wrap_180(sensor.heading - 180)
            #print sensor.heading, sensor.role, sensor.pitch
            # Gyroscope data (in degrees per second)
            #gyro = axis(bno.read_gyroscope())
            #print gyro.x, gyro.y, gyro.z
            #count = 0

        # PID for role
    
        # PID for pitch
   
        # PID for yaw
    
    
    
        # combine PID data and send to motors
    
    
    
    
        # Print everything out.
        #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))
        # Other values you can optionally read:
        # Orientation as a quaternion:
        #x,y,z,w = bno.read_quaterion()
        # Sensor temperature in degrees Celsius:
        #temp_c = bno.read_temp()
        # Magnetometer data (in micro-Teslas):
        #x,y,z = bno.read_magnetometer()
        # Gyroscope data (in degrees per second):
        #x,y,z = bno.read_gyroscope()
        #print('Gyroscope x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(x,y,z))
        # Accelerometer data (in meters per second squared):
        #x,y,z = bno.read_accelerometer()
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        #x,y,z = bno.read_linear_acceleration()
        # Gravity acceleration data (i.e. acceleration just from gravity--returned
        # in meters per second squared):
        #x,y,z = bno.read_gravity()
        # Sleep for a second until the next reading.
        time.sleep(0.02)
    

# start the controller  
flightControl()







