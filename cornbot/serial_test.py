import serial
import time

arduino=serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

while 1:
	if arduino.inWaiting!=0:
		print(arduino.readline());
		
