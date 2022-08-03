# Use to communicate between RPi and Arduino via serial
# Taylor Korte 5/13/2021

import serial
import threading
from time import time, sleep 
import struct

class CommunicationState:
	SENDING_COMMAND = 0
	WAITING_FOR_COMMAND = 1

	def toString(state):
		if state == 0:
			return "SENDING COMMAND"
		elif state == 1:
			return "WAITING FOR COMMAND"

class SerialCommunication:

	def __init__(self, arduino_path):
		
		self.ser              = serial.Serial(arduino_path, 9600, timeout = 1)
		self.communication    = CommunicationState.WAITING_FOR_COMMAND
		self.thread           = None
		self.lock             = threading.Lock()
		self.base_speed       = 175
		self.maxspeed         = 200
		self.last_rcv         = 0
		sleep(2)

	def sendSerial(self, new_list):
		'''
		Sends an integer to the arduino via serial connection by converting to a 
		byte array. The sent and recieved integers are printed and the recieved 
		integer is returned from the function. 

		Parameters
		----------
		new_list : integer
			the number that is sent via serial connection

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> my_list = 1234
		>>> s.sendSerial(my_list)
		>>> received_value = s.last_rcv 
		'''

		self.lock.acquire()
		print(f'SENDING: {new_list}')
		self.ser.write(struct.pack("<l", new_list))
		sleep(.05)
		bytes_array = bytearray(self.ser.read(4))

		while not bytes_array:
			sleep(.05)
			bytes_array = bytearray(self.ser.read(4))
		
		self.last_rcv = struct.unpack('<I', bytes_array)[0]
		print(f'RECEIVING: {self.last_rcv}')
		self.lock.release()

	def checkRecievedValue(self):
		'''
		Checks the value recieved from the arduino. If this value
		is equal to 9999 returns true, else returns false.

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> my_list = 1234
		>>> s.sendSerial(my_list)
		>>> if s.checkRecievedValue():
		>>> 	sleep(1)
		'''
		
		return self.last_rcv == 9999



	def sendString(self, str_is_left, str_motor):
		'''
		Combine robot direction and motor pwm, convert to integer, and
		begin a thread to send the command via the sendSerial function

		Parameters
		----------
		str_is_left : string
			Signifies what direction the robot will move, it is either 0 (right turn), 
			1 (left turn), or 2 (straight)

		str_motor : string
			The desired pwm to send to the robot

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path)
		>>> robot_direction = str(1)
		>>> motor_pwm = f'{75:03}
		>>> s.sendString(robot_direction, motor_pwm)
		'''

		send_string = str_is_left + str_motor
		send_string = int(send_string)
		self.thread = threading.Thread(target=self.sendSerial, args=(send_string,))
		self.thread.start() 
		#self.sendSerial(send_string)

	def send_stop(self, ticks):
		'''
		Sends command to stop the robot for number of ticks, each tick is about 
		0.01 seconds.

		Parameters
		----------
		ticks : integer
			How many times you want the robot to stop

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> myticks = 1
		>>> s.send_stop(myticks)
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND	

			for i in range(ticks):
				self.thread = threading.Thread(target=self.sendSerial,  args=(0,))
				self.thread.start()
				#self.sendSerial(0)
				sleep(.01)

			self.communication = CommunicationState.WAITING_FOR_COMMAND

	def send_left(self, ticks):
		'''
		Sends command to turn the robot left for number of ticks, each tick is 
		about 0.01 seconds. To turn 90 degrees send 29 ticks.

		Parameters
		----------
		ticks : integer
			How many times you want the robot to turn

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> myticks = 29
		>>> s.send_left(myticks)
		'''

		recieved_values = []
		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(1) + str(self.base_speed))

			for i in range(ticks):
				self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
				self.thread.start()
				#self.sendSerial(motor_command)
				sleep(.01)

			self.communication = CommunicationState.WAITING_FOR_COMMAND


	def send_right(self, ticks):
		'''
		Sends command to turn the robot right for number of ticks, each tick is 
		about 0.01 seconds. To turn 90 degrees send 29 ticks.

		Parameters
		----------
		ticks : integer
			How many times you want the robot to turn

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> myticks = 29
		>>> s.send_right(myticks)
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(0) + str(self.base_speed))

			for i in range(ticks):
				self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
				self.thread.start()
				#self.sendSerial(motor_command)
				sleep(.01)

			self.communication = CommunicationState.WAITING_FOR_COMMAND

	def send_straight(self, ticks):
		'''
		Sends command to move robot straight for number of ticks, each tick is about 
		0.01 seconds. Send 43 ticks for 3 feet and 30 ticks for 2 feet.

		Parameters
		----------
		ticks : integer
			How many times you want the robot to move forward

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> myticks = 10
		>>> s.send_straight(myticks)
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(2) + str(self.base_speed))

			for i in range(ticks):
				self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
				self.thread.start()
				#self.sendSerial(motor_command)
				sleep(.01)

			self.communication = CommunicationState.WAITING_FOR_COMMAND

	def send_back(self, ticks):
		'''
		Sends command to move robot backward for number of ticks, each tick is about 
		0.01 seconds. Send 43 ticks for 3 feet and 30 ticks for 2 feet.

		Parameters
		----------
		ticks : integer
			How many times you want the robot to move back

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> myticks = 10
		>>> s.send_back(myticks)
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(3) + str(self.base_speed))

			for i in range(ticks):
				self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
				self.thread.start()
				#self.sendSerial(motor_command)
				sleep(.01)

			self.communication = CommunicationState.WAITING_FOR_COMMAND

	def send_special(self):
		'''
		This command is used to tell the robot to line up at the crease and move 
		back one foot. Arduino will send 9999 when it is done lining up.

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> s.send_special()
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(6) + str(self.base_speed))

			while self.last_rcv != 9999:
					self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
					self.thread.start()
					#self.sendSerial(motor_command)

			self.communication = CommunicationState.WAITING_FOR_COMMAND

	def send_armClose(self):
		'''
		This command is used to tell the robot to grab the ball with the arm.  
		Arduino will send 9999 when it is done.

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> s.send_special()
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(4) + str(self.base_speed))

			while self.last_rcv != 9999:
					self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
					self.thread.start()
					#self.sendSerial(motor_command)

			self.communication = CommunicationState.WAITING_FOR_COMMAND

	def send_armOpen(self):
		'''
		This command is used to tell the robot to open the the claw and push the ball.  
		Arduino will send 9999 when it is done.

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> s.send_special()
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_command = int(str(5) + str(self.base_speed))

			while self.last_rcv != 9999:
					self.thread = threading.Thread(target=self.sendSerial, args=(motor_command,))
					self.thread.start()
					#self.sendSerial(motor_command)

			self.communication = CommunicationState.WAITING_FOR_COMMAND


	def startup(self, str_is_left, str_motor, ticks):
		'''
		Send a fast (max) pwm and progressively decrease to the desired pwm. 
		Helpful for getting the robot to start moving after it gets stuck.

		Parameters
		----------
		str_is_left : string
			Signifies what direction the robot will move, it is either 0 (right turn), 
			1 (left turn), or 2 (straight)

		str_motor : string
			The desired pwm to send to the robot

		ticks : integer
		How quickly you want the robot to get to the desired pwm. Less ticks
		means that the robot will jump from max pwm to the desired pwm very quickly

		Examples
		--------
		>>> from sendserial.sendserial import *
		>>> s = SerialCommunication(path) 
		>>> myticks = 10
		>>> robot_direction = str(2)
		>>> motor_pwm = f'{75:03}
		>>> s.startup(robot_direction, motor_pwm, myticks)
		'''

		if self.communication == CommunicationState.WAITING_FOR_COMMAND: #ensures command isn't already being sent
			self.communication = CommunicationState.SENDING_COMMAND
			motor_increment = (self.maxspeed - int(str_motor))/ticks

			for i in range(ticks):
				str_motor = int(self.maxspeed - motor_increment)
				str_motor = f'{str_motor:03}'
				self.sendString(str_is_left, str_motor)
				sleep(.01)
		
			self.communication = CommunicationState.WAITING_FOR_COMMAND




