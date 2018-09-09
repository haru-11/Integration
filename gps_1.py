# -*- coding: utf-8 -*-

import serial
import time
s = serial.Serial('/dev/serial0', 38400, timeout=10)

s.readline()
time.sleep(1.0)
#s.close()
time.sleep(1.0)
try:
	while True:
		data = s.readline()
		print data
		
except KeyboardInterrupt:
	s.close()
