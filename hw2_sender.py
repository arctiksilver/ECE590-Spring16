import socket

UDP_IP = "192.168.1.88"
UDP_PORT = 5005
MESSAGE = "F0 F0"

L_speed = 0
R_speed = 0
state = 0
input_var = 'x'

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

print "User Inputs: w = forward, a = turn left, d = turn right, d = reverse"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
	if state == 0: # stoped
		if input_var == 'w':
			L_speed += 1
			R_speed += 1
			state = 1
		elif input_var == 'a':
			R_speed += 1
			state = 2
		elif input_var == 'd':
			L_speed += 1
			state = 3
		elif input_var == 's':
			L_speed -= 1
			R_speed -= 1
			state = 4
	elif state == 1: # moving forward
		if input_var == 'w':
			L_speed += 1
			R_speed += 1
			state = 1
		elif input_var == 'a':
			R_speed += 1
			state = 2
		elif input_var == 'd':
			L_speed += 1
			state = 3
		elif input_var == 's':
			L_speed = 0
			R_speed = 0
			state = 0
	elif state == 2: # turning left
		if input_var == 'w':
			R_speed = L_speed
			state = 1
		elif input_var == 'a':
			R_speed += 1
			state = 2
		elif input_var == 'd':
			L_speed += 1
			if L_speed == R_speed:
				state = 1
			else:
				state = 3
		elif input_var == 's':
			L_speed = 0
			R_speed = 0
			state = 0
	elif state == 3: # turning right
		if input_var == 'w':
			L_speed = R_speed
			state = 1
		elif input_var == 'a':
			R_speed += 1
			if L_speed == R_speed:
				state = 1
			else:
				state = 2
		elif input_var == 'd':
			L_speed += 1
			state = 3
		elif input_var == 's':
			L_speed = 0
			R_speed = 0
			state = 0
	elif state == 4: # reverse
		if input_var == 'w':
			L_speed = 0
			R_speed = 0
			state = 0
		elif input_var == 'a':
			R_speed -= 1
			state = 4
		elif input_var == 'd':
			L_speed -= 1
			state = 4
		elif input_var == 's':
			L_speed -= 1
			R_speed -= 1
			state = 4

	if L_speed > 5:		# check speed limits
		L_speed = 5
	elif L_speed < -5:
		L_speed = -5
	if R_speed > 5:
		R_speed = 5
	elif R_speed < -5:
		R_speed = -5
	
	if L_speed == R_speed and L_speed > 0:
		state = 1
	elif L_speed == R_speed and L_speed < 0:
		state = 4

	if L_speed >= 0:	# Format command message
		L_direction = 'F'
	else:
		L_direction = 'R'

	if R_speed >= 0:
		R_direction = 'F'
	else:
		R_direction = 'R'

	MESSAGE = L_direction + str(abs(L_speed)) + ' ' + R_direction + str(abs(R_speed))
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT)) # send command message

	input_var = raw_input("Command: ") # get user input




