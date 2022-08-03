from sendserial.sendserial import *
#from pynput.keyboard import Key, Controller
#import keyboard
from time import sleep
path = '/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0'
s = SerialCommunication(path)

#keyboard = Controller()
print('Input robot direction:\n')
# while True:
	

# 	if keyboard.is_pressed('w'):
# 		print('w\n')
# 		s.send_straight(1)

# 	elif keyboard.is_pressed('d'):
# 		print('d\n')
# 		s.send_left(1)

# 	elif keyboard.is_pressed('a'):
# 		print('a\n')
# 		s.send_right(1)

# 	elif keyboard.is_pressed('s'):
# 		print('s\n')
# 		s.send_back(1)

# 	elif keyboard.is_pressed('e'):
# 		print('d\n')
# 		s.send_stop(1)

# 	sleep(0.15)

import cv2
import numpy as np


from pynput.keyboard import Key, Listener, Controller
listener = None
keyPressed = None
# cap= cv2.VideoCapture(0)
# # Set camera resolution
# cap.set(3, 240)
# cap.set(4, 144)

def on_press(key):
    try:
        print("running")
        k = key.char
        ser.write(str.encode(str(k.capitalize())+"\n"))
        print(str.encode(str(k.capitalize())+"\n"))
        time.sleep(0.2)
        ser.write(b"X\n")

    except AttributeError:
        print('special key pressed: {0}'.format(
            k))
def CheckWhichKeyIsPressed():
    global listener
    if listener == None:  
        listener = Listener(on_press=on_press,suppress=True)
        listener.start()
        
_, frame = cap.read()
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
ser.flush()

while True:
    _, frame = cap.read()
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    CheckWhichKeyIsPressed()
    

    
    if key == 27:
        break
    
cap.release()
cv2.destroyAllWindows()







