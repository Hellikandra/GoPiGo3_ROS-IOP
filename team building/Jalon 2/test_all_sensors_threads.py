from time import time, sleep
from threading import Thread, Event, get_ident
#import time
import serial
from easygopigo3 import EasyGoPiGo3
from gpiozero import DistanceSensor
import math
import numpy as np
from skimage import io

## ========== GOPIGO ============ ##
#instantiate gpg
gpg = EasyGoPiGo3(use_mutex = True)

#set speed
gpg.set_speed(50)

#reset the encoders to zero
gpg.reset_encoders()



## ============== MAP =========== ##
# map creation
gpgmap = np.ones((400,400))

# map saving function
#	x, y: 	coordinates in cm of point to put on map
#	m: 		map numpy array
def putOnMap(x, y, m):
	# change referential to upper left corner of map
	xm = round(x+200)
	ym = round(200-y)
	# check that the point is still on map
	if (xm < 400) and (xm >= 0) and (ym < 400) and (ym >= 0):
		m[round(x+200), round(200-y)] = 0


## ========== SENSORS =========== ##
# Ultrasonic sensors
US1TrigPin = "GPIO4"
US1EchoPin = "GPIO17"
US2TrigPin = "GPIO5"
US2EchoPin = "GPIO6"
US3TrigPin = "GPIO12"
US3EchoPin = "GPIO13"
US4TrigPin = "GPIO19"
US4EchoPin = "GPIO26"

US1 = DistanceSensor(US1EchoPin, US1TrigPin)
US2 = DistanceSensor(US2EchoPin, US2TrigPin)
US3 = DistanceSensor(US3EchoPin, US3TrigPin)
#US4 = DistanceSensor(US4EchoPin, US4TrigPin)

# sensor positionning
US1x = 4.8 # 48 mm
US1y = 2.5 # 25 mm
US1d = 5.412 # distance in cm
US1a = 90 # 90° clockwise to face of robot
US1g = 62.49 # angle from robot direction to sensor centre, clockwise
US2x = 4.3
US2y = 6.9
US2d = 8.13
US2a = 64
US2g = 31.93
US3x = 3.4
US3y = 4.7
US3d = 5.80
US3a = 0
US3g = -35.88
US4x = 4.0
US4y = 7.0
US4d = 8.062
US4a = -57
US4g = -29.74
LIx = 0
LIy = 6.3
LId = 6.3
LIa = 0
LIg = 0
#~ EncM = 0

# LIDAR
#definition of the serial port on the RPi
ser = serial.Serial(
	port="/dev/ttyS0", 
	baudrate = 115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

#definition of the bytes of the TFMini message (see TFMini doc p. 11)
LIHEADER1 = ''
LILIHEADER2 = ''
LIDIST_L = ''
LIDIST_H = ''
LISTRENGTH_L = ''
LISTRENGTH_H = ''
LIMODE = ''
LICHECKSUM = ''

#definition of distance and strength output
LIdistance = 0
LIstrength = 0


# ===================== DEFINES =========== 
# Compute distance
#	S in cm: distance from sensor to centre of robot
#	g in ° : angle from robot direction to sensor centre, clockwise
#	a in ° : angle from robot face to sensor face, clockwise
#	b in ° : direction or robot, clockwise
#	m in cm: the distance measured
def computeDistance(S, g, a, b, m):
	return [S * math.sin(math.radians(b-g)) + m * math.sin(math.radians(b-a)), \
	S * math.cos(math.radians(b-g)) + m * math.cos(math.radians(b-a))]

# Measure robot direction
#	computes the direction of the robot in °
def robotDirection():
	enc = gpg.read_encoders()
	b = (-enc[0] + enc[1])*0.2939
	return b

## ========== THREADS ============= ##
#time parameters
start_time = time()
runtime = 12.5 # run time of the threads, in seconds

#create an event object for triggering the "shutdown" o each thread
stop_event = Event()

#target function for US1 measuring
def measureUS1(m):
	while not stop_event.is_set():
		thread_id = get_ident() # thread ID, not used
		measUS1 = US1.distance * 100 # cm
		if (measUS1 < 99):
			#~ enc = gpg.read_encoders()
			#~ b = (enc[0] - enc[1])*0.2939
			b = robotDirection()
			[x, y] = computeDistance(US1d, US1g, US1a, b, measUS1)
			#print("US1 = {}° {}cm".format(b, measUS1))
			putOnMap(x, y, m)
		sleep(0.1)

#target function for US2 measuring
def measureUS2(m):
	while not stop_event.is_set():
		thread_id = get_ident() # thread ID, not used
		measUS2 = US2.distance * 100 # cm
		if (measUS2 <= 99):
			#~ enc = gpg.read_encoders()
			#~ b = (enc[0] - enc[1])*0.2939
			b = robotDirection()
			[x, y] = computeDistance(US2d, US2g, US2a, b, measUS2)
			#print("US2 = {}° {}cm".format(b, measUS2))
			putOnMap(x, y, m)
		sleep(0.1)

#target function for US3 measuring
def measureUS3(m):
	while not stop_event.is_set():
		thread_id = get_ident() # thread ID, not used
		measUS3 = US3.distance * 100 # cm
		if (measUS3 < 9):
			#~ enc = gpg.read_encoders()
			#~ b = (enc[0] - enc[1])*0.2939
			b = robotDirection()
			[x, y] = computeDistance(US3d, US3g, US3a, b, measUS3)
			#print("US3 = {}° {}cm".format(b, measUS3))
			putOnMap(x, y, m)
		sleep(0.1)

#target function for LI measuring
def measureLI(m):
	#reset flag
	LIreset = 0

	LIcounter = 0
	while not stop_event.is_set():
		thread_id = get_ident() # thread ID, not used
		
		# read byte on serial port
		LIrawmsg = ser.read()

		#check counter to know what byte needs to be read
		if LIcounter == 0:
			#Byte 0 : first header is expected to be 0x59
			if LIrawmsg == b'\x59':
				LIHEADER1 = LIrawmsg #store byte
				LIcounter = LIcounter + 1 #increment counter
			else:
				LIreset = 1 #raise the reset flag
		elif LIcounter == 1:
			#Byte 1 : second header is expected to also be 0x59
			if LIrawmsg == b'\x59':
				LIHEADER2 = LIrawmsg
				LIcounter = LIcounter + 1
			else:
				LIreset = 1
		elif LIcounter == 2:
			#store the low byte distance
			LIDIST_L = LIrawmsg
			LIcounter = LIcounter + 1
		elif LIcounter == 3:
			#store the high byte distance
			LIDIST_H = LIrawmsg
			LIcounter = LIcounter + 1
		elif LIcounter == 4:
			#store the low byte strength
			LISTRENGTH_L = LIrawmsg
			LIcounter = LIcounter + 1
		elif LIcounter == 5:
			#store the high byte strength
			LISTRENGTH_H = LIrawmsg
			LIcounter = LIcounter + 1
		elif LIcounter == 6:
			#mode byte (not used here)
			LIMODE = LIrawmsg
			LIcounter = LIcounter + 1
		elif LIcounter == 7:
			#byte 7 is expected to be 0x00
			if LIrawmsg == b'\x00':
				LIcounter = LIcounter + 1
			else:
				LIreset = 1
		elif LIcounter == 8:
			#checksum is not used here
			LICHECKSUM = LIrawmsg
			LIcounter = LIcounter + 1
		else:
			# should never happen
			LIreset = 1

		#if all bytes have been received, process message
		if LIcounter >= 9:
			#compute distance
			LIdistance = int.from_bytes(LIDIST_L,byteorder='big') + int.from_bytes(LIDIST_H,byteorder='big')*256
			#compute strength
			LIstrength = int.from_bytes(LISTRENGTH_L,byteorder='big') + int.from_bytes(LISTRENGTH_H,byteorder='big')*256
			#print results
			#~ enc = gpg.read_encoders()
			#~ b = (enc[0] - enc[1])*0.2939
			b = robotDirection()
			[x, y] = computeDistance(LId, LIg, LIa, b, LIdistance)
			#print("LI = {}° {}cm".format(b, LIdistance))
			putOnMap(x, y, m)
			#print('Distance=%5d cm; Strength=%5d' % (LIdistance, LIstrength))
			#reset all received bytes
			LIreset = 1

		if LIreset:
			LIcounter = 0
			LIHEADER1 = ''
			LIHEADER2 = ''
			LIDIST_L = ''
			LIDIST_H = ''
			LISTRENGTH_L = ''
			LISTRENGTH_H = ''
			LIMODE = ''
			LICHECKSUM = ''
			LIreset = 0
		

#create an object for each tread
US1thread = Thread(target=measureUS1, args=(gpgmap,))
US2thread = Thread(target=measureUS2, args=(gpgmap,))
US3thread = Thread(target=measureUS3, args=(gpgmap,))
LIthread = Thread(target=measureLI, args=(gpgmap,))
#Encthread = Thread(target=measureEncoder, args=(EncM,))



## =========== MAIN LOOP ============ ##
# start the threads
US1thread.start()
US2thread.start()
US3thread.start()
LIthread.start()
#~ Encthread.start()

#start the robot
gpg.spin_right()

#let it run for [runtime] seconds
while (time() - start_time <= runtime) and (robotDirection() < 360):
	sleep(0.1)

# then set the stop event variable
print("Stop")
stop_event.set()

#and wait both threads to end
US1thread.join()
US2thread.join()
US3thread.join()
LIthread.join()
#~ Encthread.join()

# save the map
io.imsave('map.png', gpgmap)
print("Image saved")


# stop robot if still moving
gpg.stop()
