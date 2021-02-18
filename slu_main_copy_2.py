from slu_functions import *
import time
import socket
import argparse
import json
import threading
from SbusParser import SbusParser


############################### Arguments parser #############################################

parser = argparse.ArgumentParser(description='Run line follower')
parser.add_argument('--slu',
					help="This is device name of slu unit")

args = parser.parse_args()
slu = args.slu

if slu is None:
	print("Error: please specify slu unit")
	quit()

ser = serial.Serial(slu, timeout=0)  

###################################################################################

################################## Create moab socket ##################################

SLU_MOAB_PORT = 31338
JETSON_IP = "192.168.8.26"
slu_moab_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
slu_moab_sock.bind((JETSON_IP, SLU_MOAB_PORT))
slu_moab_sock.setblocking(0)

###################################################################################

################################## Create webrtc socket ##################################

SLU_WEBRTC_PORT = 31339
JETSON_IP = "192.168.8.26"
slu_webrtc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
slu_webrtc_sock.bind((JETSON_IP, SLU_WEBRTC_PORT))
slu_webrtc_sock.setblocking(0)

###################################################################################

################################ Init #############################################

channel_trim = 1020
deadband_width = 20
speed_factor = 20
acceleration = 10000
speed_factor_webrtc = 10000

###################################################################################


time.sleep(2)

ser.write(initialize().encode())

ser.write(set_drive_mode(1).encode())

ser.write(set_velocity_absolutely(0).encode())

ser.write(set_acceleration_absolutely(acceleration).encode())


channels = SbusParser()

first = True

latest_position = 0
def read_from_serial_port(ser):
	global latest_position

	while True:
		serial_fb = ser.readline().decode()
		# print(serial_fb)
		try:
			if (len(serial_fb) > 0) and (int(serial_fb) != 0):
				latest_position = int(serial_fb)
				# print("serial_fb: " + str(serial_fb) + "          len: "+ str(len(serial_fb)))
		except:
			pass


thread = threading.Thread(target=read_from_serial_port, args=(ser,))
thread.start()

while True:
	
	pkt = None
	moab_pkt = None
	webrtc_pkt = None
	try:
		while True:
			moab_pkt, addr = slu_moab_sock.recvfrom(1024, socket.MSG_DONTWAIT)
			# pkt = data.decode()
	except:
		moab_pkt = moab_pkt

	try:
		while True:
			webrtc_pkt, addr = slu_webrtc_sock.recvfrom(1024, socket.MSG_DONTWAIT)
			#print("got imu packet")
	except:
		webrtc_pkt = webrtc_pkt


	if webrtc_pkt:
		webrtc_pkt_dec = webrtc_pkt.decode()
		dec = json.loads(webrtc_pkt_dec)
		print(dec)
		ch_right_y = dec['AXES']['#03']

		try:

			# move with right stick ch C

			if  abs(ch_right_y) > 0.05:
				# print("down")
				ser.write(set_velocity_absolutely((-ch_right_y) * speed_factor_webrtc).encode())

			else:
				print("stay")
				ser.write(set_velocity_absolutely(0).encode())

		
		except Exception as e:
			print(e)
	elif moab_pkt:
		channels.parse_packet(moab_pkt)
		print(channels.__dict__)
		ch3 = channels.prev_ch3
		ch6 = channels.prev_ch6
		try:
			if ch6 > 1500:
				ser.write(initialize().encode())
				time.sleep(2)
			# move with right stick ch C

			if  ch3 < channel_trim-deadband_width:
				# print("down")
				ser.write(set_velocity_absolutely((ch3-channel_trim) * speed_factor).encode())
			elif ch3 > channel_trim+deadband_width:
				# print("up")
				ser.write(set_velocity_absolutely((ch3-channel_trim) * speed_factor).encode())
			else:
				# print("stay")
				ser.write(set_velocity_absolutely(0).encode())

		
		except Exception as e:
			print(e)

	# ser.flush()
	ser.write(get_position().encode())
	"""serial_fb = None
	try:
		while True:

			serial_fb = ser.readline().decode()
			print(serial_fb)

			if (len(serial_fb) == 0) or (serial_fb != "0"):
				break

			# pkt = data.decode()
	except:
		serial_fb = serial_fb"""
	

	# serial_fb = ser.readline().decode()
	print(latest_position)

	time.sleep(0.05)


