#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*

# ******************************************************

# Chris Glomb
# Demo 3
# 4/24/16

# ******************************************************

import numpy as np
import sys
import time
import math
from ctypes import *
import serial
import struct

SET_POSITION = 0
GET_POSITION = 1

class CONTROLLER_REF(Structure):
    _pack_ = 1
    _fields_ = [("x", c_float), ("y", c_float), ("z", c_float)]

ser1 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)




def FK(Theta1, Theta2, Theta3):
#      Yaw   , Pitch , Bend
	x0,y0,z0 = 0,0,0.073 # limb origin
	l1 = 0.143 # lower segment length
	l2 = 0.143 # upper segment length
	t1 = Theta1 + 0 # Angle offsets
	t2 = Theta2 + 0
	t3 = Theta3 - 1.5708

	
	position = ci.CONTROLLER_REF()
	
	Rz = np.matrix(((math.cos(t1), -math.sin(t1), 0, 0), (math.sin(t1), math.cos(t1), 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
	Rx1 = np.matrix(((1, 0, 0, 0), (0, math.cos(t2), -math.sin(t2), 0), (0, math.sin(t2), math.cos(t2), 0), (0, 0, 0, 1)))
	Rx2 = np.matrix(((1, 0, 0, 0), (0, math.cos(t3), -math.sin(t3), 0), (0, math.sin(t3), math.cos(t3), 0), (0, 0, 0, 1)))

	D1 = np.matrix(((1,0,0,0),(0,1,0,l1),(0,0,1,0),(0,0,0,1)))
	D2 = np.matrix(((1,0,0,0),(0,1,0,l2),(0,0,1,0),(0,0,0,1)))

	T1 = Rx1*Rz*D1
	T2 = Rx2*D2
	
	T= T1*T2
	
	position.x = T[0,3] + x0
	position.y = T[1,3] + y0
	position.z = T[2,3] + z0
	
	#print Theta1,Theta2,Theta3
	#print T1, T2
	#print position.x,position.y,position.z
	
	return position
# end FK
	

def IK(x,y,z):
	# Get desired joint values
	ser1.flushInput();
	ser1.write(GET_POSITION);
	buff = ser1.read(8);
	pos = struct.unpack('hhhh', buff);
		
	# determine distance to target
	current = FK(pos[0],pos[1],pos[2])
	#print "current: %1.3f %1.3f %1.3f" % (current.x, current.y, current.z)
	dist = math.sqrt(((x-current.x)**2)+((y-current.y)**2)+((z-current.z)**2))
	if dist > 0.03:
		e = np.array([(x-current.x)/8,(y-current.y)/8,(z-current.z)/8])
		# compute jacobian
		delta = 0.05
		J = np.zeros(shape=(4,3))
		for i in range(0,4):
			for j in range(0,3):
				if i == 0:
					new = FK(P+delta,Y,R,B,limb)
				elif i == 1:
					new = FK(P,Y+delta,R,B,limb)
				elif i == 2:
					new = FK(P,Y,R+delta,B,limb)
				elif i == 3:
					new = FK(P,Y,R,B+delta,limb)
				if j == 0:
					J[i,j] = float(new.x - current.x)/delta
				if j == 1:
					J[i,j] = float(new.y - current.y)/delta
				if j == 2:
					J[i,j] = float(new.z - current.z)/delta
		
		Ji = np.dot(np.linalg.inv(np.dot(J.transpose(), J)), J.transpose())
		#Ji = J.transpose()
		dTheta = np.dot(e,Ji)
		for i in range(0,4):
			dTheta[i] = dTheta[i]/1.0
			if dTheta[i] > 0.2:
				dTheta[i] = 0.2
			elif dTheta[i] < -0.2:
				dTheta[i] = -0.2
		if limb == ci.R_arm:
			ref.ref[ha.RSP] = P + dTheta[0]
			ref.ref[ha.RSY] = Y + dTheta[1]
			ref.ref[ha.RSR] = R + dTheta[2]
			ref.ref[ha.REB] = B + dTheta[3]
		elif limb == ci.L_arm:
			ref.ref[ha.LSP] = P + dTheta[0]
			ref.ref[ha.LSY] = Y + dTheta[1]
			ref.ref[ha.LSR] = R + dTheta[2]
			ref.ref[ha.LEB] = B + dTheta[3]
		elif limb == ci.R_leg:
			ref.ref[ha.RHP] = P + dTheta[0]
			ref.ref[ha.RHY] = Y + dTheta[1]
			ref.ref[ha.RHR] = R + dTheta[2]
			ref.ref[ha.RKN] = B + dTheta[3]
		elif limb == ci.L_leg:
			ref.ref[ha.LHP] = P + dTheta[0]
			ref.ref[ha.LHY] = Y + dTheta[1]
			ref.ref[ha.LHR] = R + dTheta[2]
			ref.ref[ha.LKN] = B + dTheta[3]
		
		
		# SET Limits
		
# end IK

# wait for first position from controller
#c.get(pos, wait=True, last=True)		
#while 1:
#	c.get(pos, wait=False, last=True)
#	IK(pos.x, pos.y, pos.z,ci.R_arm)
#	IK(-pos.x, pos.y, pos.z,ci.L_arm)
#	SimSleep(0.1)

# Get desired joint values
ser1.flushInput();
ser1.write(GET_POSITION);
buff = ser1.read(8);
pos = struct.unpack('hhhh', buff);

print pos[0], pos[1], pos[2], pos[3]
