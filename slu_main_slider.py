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

################################## Create slider input socket ##################################

SLU_IN_PORT = 31338
JETSON_IP = "192.168.8.26"
slu_in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
slu_in_sock.bind((JETSON_IP, SLU_MOAB_PORT))
slu_in_sock.setblocking(0)

###################################################################################

################################## Create feedback publish socket ##################################

SLU_OUT_PORT = 31339
JETSON_IP = "192.168.8.26"
slu_out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


###################################################################################

################################ Init #############################################


acceleration = 10000
loop_sleep_time = 0.05

velocity_deadband = 100
position_deadband = 100

###################################################################################


time.sleep(2)

ser.write(initialize().encode())

ser.write(set_drive_mode(0).encode())  # 0 means position control

ser.write(set_velocity_absolutely(0).encode())

ser.write(set_acceleration_absolutely(acceleration).encode())


first = True

################################### Separate thread for feedback #################################

prev_time = time.time()
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

####################################################################################################

while True:
	
	webrtc_pkt = None

	try:
		while True:
			webrtc_pkt, addr = slu_webrtc_sock.recvfrom(1024, socket.MSG_DONTWAIT)
			#print("got imu packet")
	except:
		webrtc_pkt = webrtc_pkt


	if webrtc_pkt:
		# Decode the input from momo
		webrtc_pkt_dec = webrtc_pkt.decode()
		dec = json.loads(webrtc_pkt_dec)
		print(dec)

		# Assign velocity and position
		position_in = dec['CAM_POS']
		velocity_in = dec['CAM_VEL']

		try:

			# Set the velocity first and then move to position and wait for position control to complete

			if  abs(velocity_in - prev_velocity_in) > velocity_deadband:
				ser.write(set_velocity_absolutely(velocity_in).encode())

			if  abs(position_in - prev_position_in) > position_deadband:
				ser.write(set_position_absolutely(position_in).encode())
				ser.write(wait())

		
		except Exception as e:
			print(e)

		prev_position_in = position_in
		prev_velocity_in = velocity_in

	# Send command for position feedback
	ser.write(get_position().encode())
	print(latest_position)

	# Send current position to data publisher
	if ( (time.time() - prev_time) > loop_sleep_time):

		# Make json
		feedback_dict = { "CUR_POS": latest_position}
		json_data = json.dumps(feedback_dict)

		# Send with udp
		slu_out_sock.sendto(json_data.encode(),(JETSON_IP,SLU_OUT__PORT))

		prev_time = time.time()


	time.sleep(loop_sleep_time)


