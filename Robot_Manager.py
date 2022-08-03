# This script manages the robot. The robot need to push the ball through each gate. 
# Taylor Korte 5/13/2021

#import enrique's code
from internalGridClassV2 import *
#import joe's code
from preprocess_class import *

from ColorDetection import *
from sendserial.sendserial import *
from time import sleep

class Gates:
	GATE_1 = 1
	GATE_2 = 2
	GATE_3 = 3
	GATE_4 = 4
	GATE_5 = 5
	GATE_6 = 6

	def __init__(self, gate=None):
		if (gate is None):
			self.state = Gates.GATE_4
		else: 
			self.state = gate

		if self.state > Gates.GATE_1:
			self.end_gate = self.state - 1
		else: 
			self.end_gate = Gates.GATE_6

	def changeGate(self):
		temp = self.state
		if (self.state == Gates.GATE_6):
			self.state = Gates.GATE_1
		else:
			self.state += 1
		print(f"CHANGING GATES FROM: {temp} to {self.state} !")

	def getGate(self):
		return self.state

#Each state represents a step the robot needs to complete before moving to the next step. 
class States:

	FINDING_GATE = 0
	#TRANSITION: The robot locates a gate
	LINE_UP_GATE_AND_BACKUP = 1
	#TRANSITION: Arduino sends 9999 (crease line-up and back up performed)
	DETECT_GATE_NUMBER = 2
	#TRANSITION: CNN returns integer
	THROW_BALL = 3
	#TRANSITION: Arduino sends 9999 (claw opens and robot moves backward and forward to push)
	CIRCUM_GATE = 4
	#TRANSITION: Internal map says that we have gone around gate
	BALL_DETECTION = 5
	#TRANSITION: ColorDetection script returns BALL FOUND
	SECURE_BALL = 6
	#TRANSITION: Arduino returns 9999 (motor has closed the arm)
	FACE_GATE = 7
	#TRANSITION: Internal map says we are at back of the gate
	LINE_UP_GATE = 8
	#TRANSITION: Arduino returns 9999 (robot is lined up at back of card)
	FIND_NEXT_GATE = 9
	#TRANSITION: Inputted current gate and next gate into internal map. Internal map outputs that we
	#have arrived at next gate

	#return to LINE_UP_GATE_AND_BACKUP


	def __init__(self):
		self.state = States.FINDING_GATE

	def changeState(self):
		temp = self.state
		if (self.state == States.FIND_NEXT_GATE):
			self.state = States.LINE_UP_GATE_AND_BACKUP
		else:
			self.state += 1
		print(f"CHANGING STATES FROM: {temp} to {self.state} !")

	def getState(self):
		return self.state

	def __str__(self):
		if self.state == States.FINDING_GATE:
			return "FINDING GATE"
		elif self.state == States.LINE_UP_GATE:
			return "LINING UP TO GATE"
		elif self.state == States.DETECT_GATE_NUMBER:
			return "DETECTING GATE NUMBER"
		elif self.state == States.THROW_BALL:
			return "THROWING BALL"
		elif self.state == States.CIRCUM_GATE:
			return "CIRCUMNAVIGATING GATE"
		elif self.state == States.BALL_DETECTION:
			return "DETECTING BALL"
		elif self.state == States.SECURE_BALL: 
			return "SECURING BALL"
		elif self.state == States.FACE_GATE: 
			return "FACING GATE"
		elif self.state == States.LINE_UP_GATE: 
			return "LINING UP AT GATE"
		elif self.state == States.FIND_NEXT_GATE: 
			return "FINDING NEXT GATE"


#Code needs to return something that will let us change state

class Manager:

	def __init__(self):
		#initialize arduino path
		self.path = '/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0'
		#initialize serial communication
		self.s = SerialCommunication(self.path)
		#initialize Taylor's color detection code
		self.robot = Robot(self.s)
		#initilize Enrique's internal map code
		self.map = internalGrid(self.s)
		#initialize Joe's cnn code
		self.digit = Digit()
		#initialize internal state
		self.state = States()
		#initialize current gate
		self.gate = Gates()

		self.ctr = 0


	def runCNN(self):
		if (self.digit.process_webcam()):
			gate_number = digit.first
			print('The gate number is:', most_likely)
			percent_likely = digit.first_percent

			self.state.changeState()
			return gate_number

	def ballDetection(self):
		if self.robot.state != BallState.CLOSE_TO_BALL:
			self.robot.run()

			self.state.changeState() #change state to secure ball

	def stateReader(self):
		if not (self.state.getState() == States.FINDING_GATE or self.state.getState() == States.CIRCUM_GATE or self.state.getState() == States.FIND_NEXT_GATE):
			updatedState = self.map.stateReader(self.state.getState(), self.gate.getGate())
			print(updatedState) #making sure its right
			self.state.changeState()
			#self.gate.changeGate()
			return self.state.getState() #should this be self.updatedState

	# handler for 0
	def FIND_THE_GATE(self):
		#STATE 0: FINDING GATE ##IS THIS RIGHT 
		#WHEN DO I USE STATE READER
		self.map.mainFunct(self.gate.state)
		self.state.changeState()
		self.run()
		
	# handler for 1
	def LINE_UP_TO_GATE_AND_BACKUP(self):
		while self.s.last_rcv != 9999:
			#line up at gate
			self.s.send_special()
		self.s.send_stop(1)
		self.state.changeState()

	# handler for 2
	def DETECT_THE_GATE_NUMBER(self):
		#Call CNN
		gate_number = self.runCNN()

	# handler for 3
	def THROW_THE_BALL(self):
		#STATE 3: THROWING BALL
		while self.s.last_rcv != 9999:
			#Open arm and push ball
			self.s.send_armOpen()
		self.s.send_stop(1)
		self.state.changeState() 

	# handler for 4
	def GO_AROUND_THE_GATE(self):
		self.map.stateReader(self.state, self.gate)
		self.state.changeState()

	# handler for 5
	def DETECT_THE_BALL(self):
		#STATE 5: FINDING BALL
		self.ballDetection()

	# handler for 6
	def SECURE_THE_BALL(self):
		#STATE 6: SECURING BALL
		while self.s.last_rcv != 9999:
			#close arm
			self.s.send_armClose()
		self.s.send_stop(1)
		self.state.changeState()

	# handler for 7
	def FACE_THE_GATE(self):
		raise NotImplementedError

	# handler for 8
	def CALIBRATE_INTERNAL_MAP(self):
		while self.s.last_rcv != 9999:
			#line up with back of gate
			self.s.send_special()
		self.s.send_stop(1)
		self.state.changeState()

	# handler for 9
	def FIND_NEXT_GATE(self):
		self.map.stateReader(self.state, self.gate)
		self.state.changeState()
		self.gate.changeGate()

	def run(self):
		while self.ctr < 6:
			self.LINE_UP_TO_GATE_AND_BACKUP()
			self.DETECT_THE_GATE_NUMBER()
			self.THROW_THE_BALL()
			self.GO_AROUND_THE_GATE()
			self.DETECT_THE_BALL()
			self.SECURE_THE_BALL()
			self.FACE_THE_GATE()
			self.CALIBRATE_INTERNAL_MAP()
			self.FIND_NEXT_GATE()
			self.ctr += 1

		print("DONE")





manager = Manager(SerialCommunication())

manager.FIND_THE_GATE()




