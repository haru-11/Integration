import serial

s = serial.Serial('/dev/serial0', 115200, timeout=10)

s.readline()

while True:

	sentence = s.readline()
	print(sentence)
