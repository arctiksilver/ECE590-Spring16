from xinput import *
from droneInput import *
import socket
import struct
import sys, time

# control values
drone = droneInput()

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#addr = (socket.gethostbyname("PiDrone"), 5005)
addr = ("192.168.1.30", 5005)
print(addr)

joysticks = XInputJoystick.enumerate_devices()
device_numbers = list(map(attrgetter('device_number'), joysticks))
print('found %d devices: %s' % (len(joysticks), device_numbers))

j = joysticks[0]
print('using %d' % j.device_number)

@j.event
def on_axis(axis, value):
    if abs(value) < 0.2:
        value = 0
    if axis == "l_thumb_x":
        drone.yaw = value
    elif axis == "l_thumb_y":
        drone.throttle = value
    if axis == "r_thumb_x":
        drone.role = value
    elif axis == "r_thumb_y":
        drone.pitch = value

while True:
    j.dispatch_events()
    data = struct.pack('ffff', drone.throttle, drone.yaw, drone.pitch, drone.role)
    print(drone.throttle, drone.yaw, drone.pitch, drone.role)
    sock.sendto(data,addr)
    time.sleep(.1)
