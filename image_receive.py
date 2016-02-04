import socket
import sys
import Image
import time

host="192.168.1.101"
port = 5005
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((host,port))

addr = (host,port)
buf=1024

while True:
	bytes = 0
	f = open("robot_receive.jpg","wb")
	startstamp = time.time()
	data,addr = sock.recvfrom(buf)
	while(data):
		f.write(data)
		sock.settimeout(2)
		endstamp = time.time()
		bytes += len(data)
		data,addr = sock.recvfrom(buf)
		if data == "end image":
			f.close()
			rate = (bytes/(endstamp - startstamp)*8)/1000
			print "frequency = ", int(1/(endstamp - startstamp)), "Hz"
			print "rate = ", int(rate), "kbps"
			Image.open("robot_receive.jpg").show()
			break
