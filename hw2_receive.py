import socket
import time
UDP_IP = "192.168.1.88"
UDP_PORT = 5005

L_speed = 0
R_speed = 0

time.time()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
      startstamp = time.time()
      data, addr = sock.recvfrom(1024)
      donestamp = time.time()
      #
      # data should be a string format "DX DX" where X is an int 0 - 5, D = F or R
      L_string, R_string = data.split(" ", 1)
      L_speed = int(L_string[1])
      if L_string[0] == "R":
         L_speed *= -1
      
      R_speed = int(R_string[1])
      if R_string[0] == "R":
         R_speed *= -1
         
      rate = (len(data)/(donestamp - startstamp)*8)
      
      
      
      print "Output: ", L_speed, R_speed
      print "    2x: ", L_speed*2, R_speed*2
      print "rate = ", int(rate), "bps"