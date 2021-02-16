from slu_functions import *
import time
import socket
import argparse
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

ser = serial.Serial(slu)  

###################################################################################

################################## Create socket ##################################

SLU_PORT = 31338
JETSON_IP = "192.168.8.26"
slu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
slu_sock.bind((JETSON_IP, SLU_PORT))
slu_sock.setblocking(0)

###################################################################################

################################ Init #############################################

channel_trim = 1020
deadband_width = 20
speed_factor = 20
acceleration = 10000

###################################################################################


time.sleep(2)

ser.write(initialize().encode())

ser.write(set_drive_mode(1).encode())

ser.write(set_velocity_absolutely(0).encode())

ser.write(set_acceleration_absolutely(acceleration).encode())


channels = SbusParser()

first = True


while True:
	
	pkt = None
	try:
		pkt, addr = slu_sock.recvfrom(1024)
		# pkt = data.decode()
	except socket.error:
		pass


	if pkt:
		try:

			channels.parse_packet(pkt)

			print(channels.__dict__)


			# move with right stick ch C
			ch3 = channels.prev_ch3
			if  ch3 < channel_trim-deadband_width:
				print("down")
				ser.write(set_velocity_absolutely((ch3-channel_trim) * speed_factor).encode())
			elif ch3 > channel_trim+deadband_width:
				print("up")
				ser.write(set_velocity_absolutely((ch3-channel_trim) * speed_factor).encode())
			else:
				print("stay")
				ser.write(set_velocity_absolutely(0).encode())

		
		except Exception as e:
			print(e)
