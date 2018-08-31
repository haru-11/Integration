import serial
import time
s = serial.Serial('/dev/serial0', 38400, timeout=10)

s.readline()
time.sleep(1.0)
#s.close()
time.sleep(1.0)
while True:
	#s = serial.Serial('/dev/serial0', 38400, timeout=10)
	sentence = s.readline()
	print(sentence)
	print(type(sentence))
	time.sleep(0.2)
	#s.close()
	time.sleep(0.2)
