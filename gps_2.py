# -*- coding: utf-8 -*-

import serial
import time
s = serial.Serial('/dev/serial0', 38400, timeout=10)

s.readline()
time.sleep(1.0)
#s.close()
time.sleep(1.0)
def get_gps():
	i = 0
	try:
		while True:

			#s = serial.Serial('/dev/serial0', 38400, timeout=10)
			sentence = s.readline()
			print sentence
			while i < 500:
				print i
				if sentence[i] == '$':
					if sentence[i+66] == 'D' or sentence[i+65] == 'D' or sentence[i+64] == 'D':
						break
				else:
					i = i+ 1
					continue
			i = 0
			#時間
			h = int(sentence[i+7]+sentence[i+8]) + 9
			if h > 23:
				h - 24
			gps_time = str(h)+sentence[i+9]+sentence[i+10]+sentence[i+11]+sentence[i+12]+sentence[i+13]+sentence[i+14]+sentence[i+15]+sentence[i+16]

			#緯度経度
			lat = float(sentence[i+20]+sentence[i+21]) + float(sentence[i+22]+sentence[i+23]+sentence[i+24]+sentence[i+25]+sentence[i+26]+sentence[i+27]+sentence[i+28])/60.0
			lon = float(sentence[i+32]+sentence[i+33]+sentence[i+34]) + float(sentence[i+35]+sentence[i+36]+sentence[i+37]+sentence[i+38]+sentence[i+39]+sentence[i+40]+sentence[i+41])/60.0
			if sentence[i+43] == 'W':
				lon = 0.0 - lon

			#速度
			speed = float(sentence[i+45]+sentence[i+46]+sentence[i+47]+sentence[i+48]) * 1.852

			#方位
			if sentence[i+54] == ',':
				dir = sentence[i+50]+sentence[i+51]+sentence[i+52]+sentence[i+53]
			elif sentence[i+55] == ',':
				dir = sentence[i+50]+sentence[i+51]+sentence[i+52]+sentence[i+53]+sentence[i+54]
			else :
				dir = sentence[i+50]+sentence[i+51]+sentence[i+52]+sentence[i+53]+sentence[i+54]+sentence[i+55]
			#計算
			print(gps_time+","+str(lon)+","+str(lat)+","+str(speed)+","+dir)
	except ValueError:
		print("ValueError")
		get_gps()
	except IndexError:
		print("IndexError")
		get_gps()
get_gps()
