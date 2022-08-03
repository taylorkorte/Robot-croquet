# Color Detection using OpenCV
# Taylor Korte 5/13/2021
 
#Possible ways to improve: continue to edit blue_lower and blue_upper, add noise?, 
#limit number of bounding boxes and bounding box area, filter/blur, derivates of bounding boxes 
# (activity series) rate of change of activity, shape recognition - finding curves, get rid of bounding box 
# inside of bounding
 
import cv2
import numpy as np
import math
import serial
import threading
from time import time, sleep 
from scipy.interpolate import interp1d
import struct
import board
import adafruit_mpu6050
from sendserial.sendserial import *

class BallState:
	SEARCHING_FOR_BALL = 0
	LOCATED_BALL = 1
	CLOSE_TO_BALL = 2
	IDLE = 3

	def toString(state):
		if state == 0:
			return "SEARCHING FOR BALL"
		elif state == 1:
			return "LOCATED BALL"
		elif state == 2:
			return "CLOSE TO BALL"
		else:
			return "IDLE"

class CommunicationState:
	SENDING_COMMAND = 0
	WAITING_FOR_COMMAND = 1

	def toString(state):
		if state == 0:
			return "SENDING COMMAND"
		elif state == 1:
			return "WAITING FOR COMMAND"

class Robot:

	def __init__(self, serialcomms):
		self.ctr =  0
		self.K_P = .8
		self.K_I = 8
		self.K_D = .8
		self.e_prev = 0
		self.basespeed = 170
		self.path = '/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.4'
		#arduino = '/dev/cu.usbmodem00001' ##use for computer
		# self.arduino = '/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0' ##use for pi
		# self.ser     = serial.Serial(self.arduino, 9600, timeout = 1)
		# self.thread  = None
		# self.lock    = threading.Lock()
		self.s = serialcomms
		# s.last_rcv 
		
		sleep(2)

		#Accelerometer:
		# self.i2c = board.I2C() 
		# self.mpu = adafruit_mpu6050.MPU6050(self.i2c)

		# Capturing video through webcam
		self.webcam = cv2.VideoCapture(0) #use 0 for computer, 1 for pi
		# self.webcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

		self.cam_width        = self.webcam.get(cv2.CAP_PROP_FRAME_WIDTH)
		self.cam_height       = self.webcam.get(cv2.CAP_PROP_FRAME_HEIGHT)
		self.state            = BallState.SEARCHING_FOR_BALL
		self.communication    = CommunicationState.WAITING_FOR_COMMAND
		self.ERROR_TO_MOTOR   = interp1d([int((-self.cam_width / 4)), int((self.cam_width / 4))],[-100,100], bounds_error=False, fill_value=(-100, 100))
		self.buffer_length    = 100
		self.state_history    = [None]*self.buffer_length #initiates buffer list of length buffer_length
		self.dt               = 0
		self.frame_history    = [None]*self.buffer_length #contour found, skips if no contour found
		self.contour_location = []
		self.refresh_rate     = 30 # could be 60
		self.command_history  = [None]*self.buffer_length
		self.maxspeed         = 200

	#Checks if robot is stuck by comparing current frame w to previous frame w
	#If robot is not moving, increases pwm by 10 for i amount of ticks
	def isRobotMoving(self, history, str_is_left, str_motor, ticks):

		history = self.sortHistory(history)

		if self.checkHistory(history, self.is_, None, is_all=False):
			return

		elif history[0][2] == history[int(self.buffer_length/2)][2]:
			if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
				self.communication = CommunicationState.SENDING_COMMAND
				self.command_history[self.ctr % self.buffer_length] = self.communication
			
				for i in range(ticks):
					str_motor = int(str_motor) + 10
					str_motor = f'{str_motor:03}'
					self.s.sendString(str_is_left, str_motor)
					sleep(.01)
				# self.thread.join()

				self.communication = CommunicationState.WAITING_FOR_COMMAND
			self.command_history[self.ctr % self.buffer_length] = self.communication

	def equal(self,a,b):
		return a == b

	def is_(self, a, b):
		return a is b

	# def checkFrameHistory(self, history):
	# 	centroids = [((x + w/2), (y+h/2)) for x,y,w,h in history]
	# 	centroid = [(sum(x)/len(centroids), sum(y)/len(centroids)) for x,y in centroids]

	# 	#Case 1: Ball disappears to left 
	# 	if centroid[0] < self.cam_width / 4:
	# 		return 1
	# 	#Case 2: Ball disappears to right
	# 	if centroid[0] > self.cam_width / 4:
	# 		return 2
	# 	#Case 3: No History of Ball 
	# 	else:
	# 		return 3


	# takes in a history and checks to see if the list fits the criteria using the 
	# operator
	def checkHistory(self, history, op, criteria, is_all=True):
		if is_all:
			return all([op(h, criteria) for h in history])
		else:
			return any([op(h, criteria) for h in history])

	#splits the list at the index (most recent frame) and flips the list order 
	#(olders frame -> newest frame) and then flips to have the newest frames 
	#at the front of the list
	#e.g [5,6,7,1,2,3,4] -> [7,6,5,4,3,2,1]
	def sortHistory(self, history):
		index = self.ctr % self.buffer_length
		if self.ctr > self.buffer_length and index != self.buffer_length - 1:
			front = history[:index]
			back = history[index:]
			history = back + front
			return history[::-1]
		else:
			return history[::-1]

	# Utilizes P, D, and I to convert distance of contour from center 
	# to a PWM motor speed (translated into a 4-digit protocol with
	# left, straight, and right encoded)
	def pid_controller(self, setpoint, area):

		e = (self.cam_width / 4) - setpoint 
		raw_error = (self.cam_width / 4) - setpoint 
		e = int(abs(self.ERROR_TO_MOTOR(e))) 
		# calculate raw error 
		# DT = Time between frames --> Example: .02 seconds
		# Frame rate --> Frequency (1 / s)
		# u = kp * e + kd (( e - edt ) / dt) 
		# if we divide by .013, we're dividing by seconds, while we need 
		# to divide by 1 / .013, or by the frequency --> 60 Hertz
		u = self.K_P * e + (self.K_D * ((e - self.e_prev) * self.dt))
		# print(f"{setpoint} RAW DISTANCE: {raw_error} CONVERSION: {e} U: {u}")
		#PWM signal
		# add basespeed to pwm speed 
		to_motor = int(self.basespeed + u)
		#encoded string 
		str_motor = f'{to_motor:03}'
		# print(f'{u} -> {to_motor}')
		#if u>0 left, u<0 right

		# If the raw error is less than 0 (to the left of the screen)
		# the integer is 1, else its 0 
		is_left = int(raw_error > 0)

		#straight
		# check if the error is within a distance of 10 from the center
		# of the screen (minor threshold). If so, then encode a straight
		# command
		if (abs(raw_error) < 20): 
			str_is_left = "2"
			#scale by area
			# Use a KP controller based on the area of the ball 
			# and subtract it from a base speed of 150 
			# If the area is large, the speed becomes slower,
			# if the area is small, the speed is greater 
			# Since the area could be larger than 150, we take the 
			# largest value --> 150 - (.005 *  area) OR 80
			to_motor = max(int(110 - (.005 * area)), 85)
			str_motor = f'{to_motor:03}'
		else:
			str_is_left = str(is_left)

		self.e_prev = e
		return str_is_left, str_motor

	def bangbang(self, setpoint, area):
		e = (self.cam_width / 4) - setpoint 
		is_left = int(e > 0)
		to_motor = int(160)
		str_motor = f'{to_motor:03}'
		if (abs(e) < 40):
			str_is_left = "2"
			to_motor = int(160)
			str_motor = f'{to_motor:03}'

		else:
			str_is_left = str(is_left)

		sleep(.25)

		return str_is_left, str_motor


	#scales frame 
	def getFrame(self):
		_, frame = self.webcam.read()

		#Resize/Scale frame:
		scale_percent = .5 # percent of original size
		width = int(self.cam_width * scale_percent)
		height = int(self.cam_height * scale_percent)
		dim = (width, height)
		frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
		return frame

	#makes color mask
	def makeMask(self, frame):
		blurred = cv2.GaussianBlur(frame, (13, 13), 0)
		hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		# Set range for blue color and define mask
		blue_lower = np.array([70, 50, 30], np.uint8)
		blue_upper = np.array([255, 250, 130], np.uint8)
		# blue_lower = np.array([75,50,50])
		# blue_upper = np.array([130,255,255])
		# construct a mask for the color then perform
		# dilations and erosions to remove any small
		# blobs left in mask

		mask = cv2.inRange(hsv, blue_lower, blue_upper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		return mask

	#finds all blue contours
	def findContours_if_any(self, mask):
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
											cv2.CHAIN_APPROX_SIMPLE)
		return contours

	#removes contours that are too big/small and are not squares
	def checkContours(self, contours):
		#check than contour is not too big or too small
		low  = 200
		high = 5000
		filtered_contours = [c for c in contours if cv2.contourArea(c) < high and cv2.contourArea(c) > low]
		#check that contour is a square
		filtered_contours = [c for c in filtered_contours if abs(cv2.boundingRect(c)[2] - cv2.boundingRect(c)[3]) < 10]
		return filtered_contours 

	#shows ball contour on frame and records location
	def findBallContour(self, filtered_contours, frame):

		contour = max(filtered_contours, key=cv2.contourArea)
		area = cv2.contourArea(contour)
		
		x, y, w, h = cv2.boundingRect(contour)
		self.contour_location = [x, y, w, h]
		self.frame_history[self.ctr % self.buffer_length] = self.contour_location

		self.state = BallState.LOCATED_BALL
		BallState.toString(self.state)
		self.state_history[self.ctr % self.buffer_length] = self.state

		imageFrame = cv2.rectangle(frame, (x, y),
								   (x + w, y + h),
								   (255, 0, 0), 2)
		
		cv2.putText(frame, "Blue", (x, y),
					cv2.FONT_HERSHEY_SIMPLEX,
					1.0, (255, 0, 0))

		setpoint = x + w/2
		
		return setpoint, frame, area

	#displays frame with ball contour
	def showFrame(self, frame):
		cv2.imshow("BALL", frame)
	

	# def overrideCommand(self, command):
	# 	# if communicationstate is currently sending, finish it's last iteration
	# 	# and immediately send the next command

	def run(self):

		while self.state != BallState.CLOSE_TO_BALL:
			# print(BallState.toString(self.state))

			start_time = time()
			frame = self.getFrame()
			mask = self.makeMask(frame)
			contours = self.findContours_if_any(mask)
			filtered_contours = self.checkContours(contours)
			
			if (len(filtered_contours) > 0):
				setpoint, frame, area = self.findBallContour(filtered_contours, frame)



				if threading.active_count() < 4:
					#str_is_left, str_motor = self.pid_controller(setpoint, area)
					str_is_left, str_motor = self.bangbang(setpoint, area)
					if self.ctr % 2 == 0:
						self.s.send_stop(1)
				#every other ish send stop.- boolean t/f 
				#print("this is the recieved val", self.s.last_rcv)
				# if self.ctr % 10 == 0:
				# 	self.s.startup(str_is_left, str_motor, 3)	
					self.s.sendString(str_is_left, str_motor)
					self.isRobotMoving(self.frame_history, str_is_left, str_motor, 5)

			elif self.communication == CommunicationState.WAITING_FOR_COMMAND and threading.active_count() < 2:
				self.state = BallState.SEARCHING_FOR_BALL
				self.state_history[self.ctr % self.buffer_length] = self.state
				# print(BallState.toString(self.state))
				if threading.active_count() < 4:
					self.s.send_left(1)
					self.s.send_stop(1)
					sleep(.1)


				# self.send_stop(1)

			# 	case = self.checkFrameHistory(self.frame_history)

			# 	self.communication = CommunicationState.SENDING_COMMAND
			# 	print(CommunicationState.toString(self.communication))

			# 	if case == 1:
			# 		self.send_left(2)
			# 		self.send_stop(2)
			
			# 	elif case == 2:
			# 		self.send_right(2)
			# 		self.send_stop(2)
					
			# 	elif case == 3:
			# 		self.send_left(2)
			# 		self.send_stop(2)

			self.showFrame(frame)
			self.ctr += 1
			self.dt = time() - start_time
			print(self.state)

			
			if self.checkHistory(self.state_history, self.equal, BallState.LOCATED_BALL):
				self.state = BallState.CLOSE_TO_BALL
				self.state_history[self.ctr % self.buffer_length] = self.state
				print(BallState.toString(self.state))

			if cv2.waitKey(10) & 0xFF == ord('q'):
				cv2.videoCapture.release()
				cv2.destroyAllWindows()
				break

		# run the motor forward for a little bit and set to idle	
		if self.state == BallState.CLOSE_TO_BALL:

			self.s.send_straight(10)
			self.s.send_stop(1)

			self.state = BallState.IDLE
			self.state_history[self.ctr % self.buffer_length] = self.state
			print(BallState.toString(self.state))

	

robot = Robot(SerialCommunication('/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0'))
robot.run()




