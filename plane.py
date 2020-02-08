import serial # import module for serial communcation
import pyautogui # import module for keyboard/mouse control
import io # import module to to use universal newline mode
import time

alive = True

# open Serial Port 5
ser = serial.Serial()
ser.baudrate = 9600 # baud rate
ser.port = 'COM13'# set port
ser.timeout = 0.15 # set timeout in seconds
ser.open() # open serial communication
ser.bytesize =serial.EIGHTBITS 
print('COM 13 Open: ', ser.is_open)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
ser.flush() # wait until all data is written

pyautogui.FAILSAFE = False # set failsafe for manual exit
width, height = pyautogui.size() # set size of screen
#print('Press Ctrl-C to quit.')
pyautogui.moveTo(680,383) # set mouse to middle


throttle_prev = 0
count = 0

while (alive): # kill switch not asserted
    line = sio.readline()
    data = line.split()
#    print (len(data))
    print(data)
    if len(data) == 7:

        # roll and pitch

        # get sign of roll and pitch
        if data[0] == "1":
            x = -int(data[1])
        else:
            x = int(data[1])
        if data[2] == "1":
            y = -int(data[3])
        else:
            y = int(data[3])

        if x>90:
            x = 90
        if x<-90:
            x = -90
        if y>90:
            y = 90
        if x<-90:
            x = -90
        xn = y*10 + 900 + 10
        yn = -1*x*10 + 540  - 5*3
        
        if(count == 0):
            pyautogui.moveTo(xn, yn)
            print("xn :",xn)
            print("yn :",yn)
        count = 0

            # throttle
        throttle = int(data[4])
        if (throttle-throttle_prev)>0:
            pyautogui.press('c')
            pyautogui.keyDown('pageup')
            pyautogui.keyUp('pageup')
        if (throttle-throttle_prev)<0:
            pyautogui.keyDown('pagedown')
            pyautogui.keyUp('pagedown')
        throttle_prev = throttle


		#wheel brake
        left_brake = int(data[5])
        if left_brake == 1:
            pyautogui.keyDown(',')
        elif left_brake ==2:
            pyautogui.keyUp(',')
            

        right_brake = int(data[6])

        if right_brake == 1:
            pyautogui.keyDown('.')
        elif right_brake ==2:
            pyautogui.keyUp('.')
			
		#landing_gear
		landing_gear= int(data[7])
        if right_brake == 1:
            pyautogui.keyDown('g')
        elif right_brake ==2:
            pyautogui.keyUp('g')
		
        


    