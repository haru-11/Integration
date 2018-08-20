# -*- coding: utf-8 -*-

import serial
import micropyGPS
import threading
import math
import time
import pigpio
import os
import mpu9250_1
import numpy as np

pin_sb = 22
gpio_pin0 = 12
in1 = 27
in2 = 17
led1 = 25
led2 = 24
led3 = 23
led4 = 18
end = 21
x_angle = 0.0

pi = 3.1415926535
goal_latitude = 35.7150216667 	#ゴールの緯度（10進法，南緯は負の数）
goal_longitude = 139.760648333	#ゴールの経度（10進法，西経は負の数）
radius = 6378.137	#地球の半径km

gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する。
                                     # 引数はタイムゾーンの時差と出力フォーマット
mpu = mpu9250_1.SL_MPU9250(0x68,1)

mpu.resetRegister()
mpu.powerWakeUp()
mpu.setAccelRange(8,True)
mpu.setGyroRange(1000,True)
mpu.setMagRegister('1000Hz','16bit')

pi = pigpio.pi()
pi.set_mode(gpio_pin0, pigpio.OUTPUT)
pi.set_mode(pin_sb, pigpio.OUTPUT)
pi.set_mode(in1, pigpio.OUTPUT)
pi.set_mode(in2, pigpio.OUTPUT)
pi.set_mode(led1, pigpio.OUTPUT)
pi.set_mode(led2, pigpio.OUTPUT)
pi.set_mode(led3, pigpio.OUTPUT)
pi.set_mode(led4, pigpio.OUTPUT)
pi.set_mode(end, pigpio.INPUT)
pi.set_pull_up_down(end, pigpio.PUD_UP)

pi.set_PWM_frequency(pin_sb, 800)
pi.set_PWM_range(pin_sb, 255)
pi.set_PWM_dutycycle(pin_sb, 0)

#pi.hardware_PWM(gpio_pin0, 50,( 0.15/20.0) * 1000000)

pi.write(in1, 0)
pi.write(in2, 0)
pi.write(led1, 1)
pi.write(led2, 0)
pi.write(led3, 0)
pi.write(led4, 0)

time.sleep(1.0)
s = serial.Serial('/dev/serial0', 115200, timeout=10)
s.write(b"serial ok!\r\n")
x_angle = 0

f = open('test2.txt','a')
f.write(b"ok\r\n".decode('utf-8'))
f.close()
class MCP3002:

	def __init__(self):
		pi = pigpio.pi()
		self.h = pi.spi_open(0, 75000, 0)

	def get_ADC(self):
		c, d =pi.spi_xfer(self.h,[0x68,0x00])
		return (d[0]<<8)+d[1]

adc =  MCP3002()

def cal_gps(radius, goal_latitude, goal_longitude, now_lat, now_lon):
	#度をラジアンに変換
	delta_lon = math.radians(goal_longitude - now_lon)
	now_lat = math.radians(now_lat)
	now_lon = math.radians(now_lon)
	goal_latitude = math.radians(goal_latitude)
	goal_longitude = math.radians(goal_longitude)

	#距離方位角計算 https://keisan.casio.jp/exec/system/1257670779
	dist = radius * math.acos(math.sin(goal_latitude)*math.sin(now_lat) + math.cos(goal_latitude)*math.cos(now_lat)*math.cos(delta_lon))
	azi = (180.0/3.1415926535)*math.atan2(math.sin(delta_lon), math.cos(now_lat)*math.tan(goal_latitude) - math.sin(now_lat)*math.cos(delta_lon))
	if azi < 0:
		azi = azi + 360.0
	return dist, azi

def get_gps():
	global gps_time, lat, lon, speed, dir, dist, azi
	s.readline()
	while True:

		sentence = s.readline().decode('utf-8')
		if sentence[0] != '$':
			continue
		#時間
		h = int(sentence[7]+sentence[8]) + 9
		if h > 23:
			h - 24
		gps_time = str(h)+sentence[9]+sentence[10]+sentence[11]+sentence[12]+sentence[13]+sentence[14]+sentence[15]+sentence[16]

		#緯度経度
		lat = float(sentence[20]+sentence[21]) + float(sentence[22]+sentence[23]+sentence[24]+sentence[25]+sentence[26]+sentence[27]+sentence[28])/60
		lon = float(sentence[32]+sentence[33]+sentence[34]) + float(sentence[35]+sentence[36]+sentence[37]+sentence[38]+sentence[39]+sentence[40]+sentence[41])/60
		if sentence[43] == 'W':
			lon = 0.0 - lon

		#速度
		speed = float(sentence[45]+sentence[46]+sentence[47]+sentence[48]) * 1.852

		#方位
		if sentence[55] == ',':
			dir = sentence[50]+sentence[51]+sentence[52]+sentence[53]+sentence[54]
		else :
			dir = sentence[50]+sentence[51]+sentence[52]+sentence[53]+sentence[54]+sentence[55]
		#計算
		dist, azi = cal_gps(radius, goal_latitude, goal_longitude, lat, lon)

gpsthread = threading.Thread(target=get_gps, args=()) # 上の関数を実行するスレッドを生成
gpsthread.daemon = True
gpsthread.start() # スレッドを起動

def get_axis():
	global x_angle
	while True:

		i = 0
		data_accx = [0]
		data_accy = [0]
		data_accz = [0]
		while True:
			now=time.time()
			acc=mpu.getAccel()
			data_accx.append(acc[0])
			data_accy.append(acc[1])
			data_accz.append(acc[2])

			if i == 10:
				medianx = np.median(data_accx)
				mediany = np.median(data_accy)
				medianz = np.median(data_accz)
				break
			i = i + 1
			sleepTime = 0.05 - (time.time() - now)
			if sleepTime < 0.0:
				continue
			time.sleep(sleepTime)
		#gyr = mpu.getGyro()
		#mag = mpu.getMag()

		#x_angle = math.degrees( math.atan2( acc[0], math.sqrt(acc[1] ** 2 + acc[2] ** 2 )))
		#y_angle = math.degrees( math.atan2( acc[1], math.sqrt(acc[0] ** 2 + acc[2] ** 2 )))
		x_angle = math.degrees( math.atan2( medianx, math.sqrt(mediany ** 2 + medianz ** 2 )))
		if medianz>0:
			x_angle = (90-x_angle) + 90
		y_angle = math.degrees( math.atan2( mediany, math.sqrt(medianx ** 2 + medianz ** 2 )))
#		print('x:'+ str(x_angle))
#       	print('y:'+ str(y_angle))
#       	print('s:'+ str(sita))
#       	print ('%+8.7f,%+8.7f,%+8.7f' % (mag[0],mag[1],mag[2]))
#       	time.sleep(1.0)


mputhread = threading.Thread(target=get_axis, args=()) # 上の関数を実行するスレッドを生成
mputhread.daemon = True
mputhread.start() # スレッドを起動

def cb_interrupt(gpio, level, tick):
	print('THE END')
	s.write(b"THE END\r\n")
	print((gpio, level, tick))
	f.close()
	pi.set_PWM_dutycycle(pin_sb, 0)
	pi.write(led1, 0)
	pi.write(led2, 0)
	pi.write(led3, 0)
	pi.write(led3, 0)
	os._exit(1)

i = 3
while i > 0:
        d = adc.get_ADC()
        s.write(str(i).encode()+b"sec,"+str(d).encode()+b"\r\n")
        i = i - 1
        time.sleep(1)
while True:
        d = adc.get_ADC()
        #s.write(str(d)+"\r\n")
        if d > 0:
                break
        time.sleep(1.0)
i=3
while i > 0:
        d = adc.get_ADC()
        s.write(str(i).encode()+b"sec,"+str(d).encode()+b"\r\n")
        i = i - 1
        time.sleep(1)

f = open('test2.txt','a')

s.write(b"release\r\n")
pi.write(in1, 0)
pi.write(in2, 1)

i = 1
while i > 0:
	pi.write(in1, 0)
	pi.write(in2, 1)
	pi.set_PWM_dutycycle(pin_sb, 255)
	time.sleep(1.0)
	pi.set_PWM_dutycycle(pin_sb, 0)
	pi.write(in1, 1)
	pi.write(in2, 1)
	time.sleep(1.0)
	pi.write(in1, 1)
	pi.write(in2, 0)
	pi.set_PWM_dutycycle(pin_sb, 255)
	time.sleep(1.0)
	pi.set_PWM_dutycycle(pin_sb, 0)
	pi.write(in1, 1)
	pi.write(in2, 1)
	time.sleep(1.0)
	i = i - 1
	s.write(b'RELEASE_MODE'+str(i).encode()+b'\r\n')

i = 50
pi.write(in1, 0)
pi.write(in2, 1)
servo = 0

try:
	while True:
		now = time.time()
		cb = pi.callback(end, pigpio.FALLING_EDGE, cb_interrupt)

		if x_angle < 90:
			i= i+8
			if i > 250:
				i = 255
			pi.set_PWM_dutycycle(pin_sb, i)
		elif x_angle > 90 and x_angle < 200:
			i = i-10
			if i < 0:
				i = 50
			pi.set_PWM_dutycycle(pin_sb, i)
		else:
			pi.write(in1, 0)
			pi.write(in2, 0)
			time.sleep(1)
			pi.write(in1, 0)
			pi.write(in2, 0)

		if speed > 1.5:
			pi.set_PWM_dutycycle(pin_sb, 200)
			pi.write(led3, 1)
			if float(dir) - azi < 10:
				pi.hardware_PWM(gpio_pin0, 50,( 1.7/20.0) * 1000000)
				servo = 1.7
			elif float(dir) - azi < -10:
				pi.hardware_PWM(gpio_pin0, 50,( 1.3/20.0) * 1000000)
				servo = 1.3
			else:
				pi.hardware_PWM(gpio_pin0, 50,( 1.5/20.0) * 1000000)
				servo = 1.5
		if dist < 0.01:
			pi.write(in1, 0)
			pi.write(in2, 0)
			pi.set_PWM_dutycycle(pin_sb, 0)
			s.write(b"GOAL!!!\r\n")
			print("goal!!")
			f.write(b'goal\r\n')
			f.close()
			pi.write(led1, 0)
			pi.write(led2, 0)
			pi.write(led3, 0)
			pi.write(led3, 0)
		print("To goal:"+str(dist)+"[m]")
		print("To goal:"+str(azi)+"[deg]")
		print("now:"+str(speed)+"[m/s]")
		print("now:"+dir+"[deg]")
		print("x_angle:"+str(x_angle))
		print("duty:"+ str(i))
		#print((str(num[4])+','+str(num[5])))
		f.write(gps_time+','+str(lat)+','+str(lon)+','+str(speed)+','+str(dir)+','+str(x_angle)+','+str(i)+','+str(servo)+'\r\n')
		s.write(str(lat).encode()+b','+str(lon).encode()+b'\r\n')
		time.sleep(0.2)
		s.write(str(dir).encode()+b','+str(x_angle).encode()+b','+str(i).encode()+b','+str(servo).encode()+b'\r\n')
		sleepTime = 0.5 - (time.time() - now)
		if sleepTime < 0.0:
			continue
		time.sleep(sleepTime)
except KeyboardInterrupt:
			pi.set_PWM_dutycycle(pin_sb, 0)
			pi.write(led1, 0)
			pi.write(led2, 0)
			pi.write(led3, 0)
			pi.write(led3, 0)
			f.close()
			s.close()
			s.write(b"KeyboardInterrupt,close(serial)\r\n")
			pass
