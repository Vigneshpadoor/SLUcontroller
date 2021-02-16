import json
import socket
import struct
import configparser
import time
from retry import retry
import timeout_decorator
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import numpy as np
import transforms3d

from ImuPacket import ImuPacket


imu = ImuPacket()
ini_config = configparser.ConfigParser()
ini_config.read('FARBOT.ini')

section = 'moab'
MOAB_COMPUTER = str(ini_config.get(section, 'MOAB_COMPUTER'))
MOAB_PORT = int(ini_config.get(section, 'MOAB_PORT'))
#SBUS_PORT = int(ini_config.get(section, 'SBUS_PORT'))

section = 'machine'
MAX_CHASE_SPEED    = float(ini_config.get(section, 'MAX_CHASE_SPEED'))

# This is the sbus udp port.
sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sbus_sock.bind(("0.0.0.0", 0))

"""darknet_farbot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
darknet_farbot_sock.bind(("192.168.8.26", 50000))"""

BNO055_RX_PORT = 27114
imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
imu_sock.bind(("0.0.0.0", BNO055_RX_PORT))

################################## controller socket ##################################

CONTROLLER_PORT = 31339
JETSON_IP = "192.168.8.26"
controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

###################################################################################

#axes0:Lスティック左,右(-1 <= 0.00 >= +1)
#axes1:Lスティック上,下(-1 <= 0.00 >= +1)
#ニュートラルポジション帯
MAX_DEADBAND = 0.1
MIN_DEADBAND = -0.1

#スティックの最大値,最小値
MAX_STICK = 1
MIN_STICK = -1


def SendFloat(float1, float2):
    udpPacket = struct.pack('ff', float1, float2)
    sbus_sock.sendto(udpPacket, (MOAB_COMPUTER, MOAB_PORT))

def map(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def map2(val1, val2, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def Neutral_pos(axes):
    if (axes <= MAX_DEADBAND and axes >= MIN_DEADBAND):
        return True
    else:
        return False

def moter_rpm(axes0,axes1):
    MotorRPM_L = None
    MotorRPM_R = None

    #ニュートラルポジション
    if (Neutral_pos(axes0) and Neutral_pos(axes1)):
        MotorRPM_L = 0.0
        MotorRPM_R = 0.0
        return MotorRPM_L, MotorRPM_R
    #前後進
    elif(Neutral_pos(axes0)):
        MotorRPM_L = map(axes1, MIN_STICK, MAX_STICK, -MAX_CHASE_SPEED, MAX_CHASE_SPEED)
        MotorRPM_R = MotorRPM_L
        return MotorRPM_L, MotorRPM_R
    #旋回
    elif(Neutral_pos(axes1)):
        MotorRPM_L = map(axes0, MIN_STICK, MAX_STICK, -MAX_CHASE_SPEED/2, MAX_CHASE_SPEED/2)
        MotorRPM_R = -MotorRPM_L
        return MotorRPM_L, MotorRPM_R
    #カーブ
    else:
        if(axes0 > MAX_DEADBAND and axes1 > MAX_DEADBAND):
            SCALE_X = map(axes0, MAX_DEADBAND, MAX_STICK, 0, MAX_CHASE_SPEED/2)
            SCALE_Y = map(axes1, MAX_DEADBAND, MAX_STICK, 0, MAX_CHASE_SPEED)
            MotorRPM_L = SCALE_X + SCALE_Y 
            MotorRPM_R = SCALE_Y - SCALE_X
            if abs(MotorRPM_L) >= MAX_CHASE_SPEED:
              MotorRPM_L = MAX_CHASE_SPEED

        elif(axes0 < MIN_DEADBAND and axes1 > MAX_DEADBAND):
            SCALE_X = map(axes0,  MAX_DEADBAND, MAX_STICK, 0, MAX_CHASE_SPEED/2)
            SCALE_Y = map(axes1,  MAX_DEADBAND, MAX_STICK, 0, MAX_CHASE_SPEED)
            MotorRPM_L = SCALE_X + SCALE_Y 
            MotorRPM_R = SCALE_Y - SCALE_X
            if abs(MotorRPM_R) >= MAX_CHASE_SPEED:
               MotorRPM_R = MAX_CHASE_SPEED

        elif(axes0 > MAX_DEADBAND and axes1 < MIN_DEADBAND):
            SCALE_X = map(axes0,  MIN_DEADBAND, MIN_STICK, 0, -MAX_CHASE_SPEED/2)
            SCALE_Y = map(axes1,  MIN_DEADBAND, MIN_STICK, 0, -MAX_CHASE_SPEED)
            MotorRPM_L = SCALE_X + SCALE_Y 
            MotorRPM_R = SCALE_Y - SCALE_X
            if abs(MotorRPM_R) >= MAX_CHASE_SPEED:
               MotorRPM_R = -MAX_CHASE_SPEED

        elif(axes0 < MIN_DEADBAND and axes1 < MIN_DEADBAND):
            SCALE_X = map(axes0, MAX_DEADBAND, MAX_STICK, 0, MAX_CHASE_SPEED/2)
            SCALE_Y = map(axes1, MAX_DEADBAND, MAX_STICK, 0, MAX_CHASE_SPEED)
            MotorRPM_L = SCALE_X + SCALE_Y 
            MotorRPM_R = SCALE_Y - SCALE_X
            if abs(MotorRPM_L) >= MAX_CHASE_SPEED:
               MotorRPM_L = -MAX_CHASE_SPEED
        
        return MotorRPM_L, MotorRPM_R

def turn_180(turn_angle, direction, pkt):
    global turn
    global init_angle
    global target_angle
    global target_angle_half
    imu.parse(pkt)
    rot = transforms3d.quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])
    angleY = -np.arctan2(rot[1, 0], rot[0,0])
    print("Current Angle :" + str(angleY))
    print("Target Angle :" + str(target_angle))
    
    if direction == 'left':
        if turn == "no_turn":
            turn = "left_180_turn"
            init_angle = angleY
            target_angle = angleY - turn_angle
            target_angle_half = angleY - turn_angle/2
            print("Target Angle :" + str(target_angle))

        # print(target_angle)
        # print(angleY)
        # print(angleY > target_angle)
        # print(angleY < target_angle + 2 * np.pi)

        if ((angleY <= init_angle + 0.1) and (angleY > target_angle_half)) or (angleY > target_angle_half + 2 * np.pi):
            return 0, -10
        elif ((angleY <= init_angle + 0.1) and (angleY > target_angle)) or (angleY > target_angle + 2 * np.pi):
            return 10, 0
        else:
            turn = "no_turn"
            return 0, 0

    if direction == 'right':
        if turn == "no_turn":
            turn = "right_180_turn"
            init_angle = angleY
            target_angle = angleY + turn_angle
            target_angle_half = angleY + turn_angle/2
            print("Target Angle :" + str(target_angle))

        # print(angleY)
        if ((angleY >= init_angle - 0.1) and (angleY < target_angle_half)) or (angleY < target_angle_half - 2 * np.pi):
            return -10, 0
        if ((angleY >= init_angle - 0.1) and (angleY < target_angle)) or (angleY < target_angle - 2 * np.pi):
            return 0, 10
        else:
            turn = "no_turn"
            return 0, 0

def turn_90(turn_angle, direction, pkt):
    global turn
    global init_angle
    global target_angle
    imu.parse(pkt)
    rot = transforms3d.quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])
    angleY = -np.arctan2(rot[1, 0], rot[0,0])
    print("Current Angle :" + str(angleY))
    print("Target Angle :" + str(target_angle))
    
    if direction == 'left':
        if turn == "no_turn":
            turn = "left_turn"
            init_angle = angleY
            target_angle = angleY - turn_angle
            print("Target Angle :" + str(target_angle))

        # print(target_angle)
        # print(angleY)
        # print(angleY > target_angle)
        # print(angleY < target_angle + 2 * np.pi)
        if ((angleY <= init_angle + 0.1) and (angleY > target_angle)) or (angleY > target_angle + 2 * np.pi):
            return 0, -10
        else:
            turn = "no_turn"
            return 0, 0

    if direction == 'right':
        if turn == "no_turn":
            turn = "right_turn"
            init_angle = angleY
            target_angle = angleY + turn_angle
            print("Target Angle :" + str(target_angle))

        # print(angleY)
        if ((angleY >= init_angle - 0.1) and (angleY < target_angle)) or (angleY < target_angle - 2 * np.pi):
            return -10, 0
        else:
            turn = "no_turn"
            return 0, 0


@retry(tries=5)
@timeout_decorator.timeout(0.1, timeout_exception = StopIteration, exception_message="socat time out")
def read_socat(term):
    read = term.readline().decode()
    return read

rpmRs, rpmLs = 0, 0
pkt = []
turn = "no_turn"
init_angle = 0
target_angle = 0
target_angle_half = 0
with open("/dev/pts/0", "rb", buffering=0) as term:
    str_buffer = ''
    rpmL = 0.0
    rpmR = 0.0
    old_stop_button = 0
    old_left_button = 0
    old_right_button = 0
    while True:

        """try:
            while True:
                data, fromAddr = darknet_farbot_sock.recvfrom(1024, socket.MSG_DONTWAIT)
                rpmRs, rpmLs = data.decode().split("and")
        except:
            rpmRs, rpmLs = rpmRs, rpmLs
            # print('except1')"""

        try:
            while True:
                pkt, fromAddr = imu_sock.recvfrom(128, socket.MSG_DONTWAIT)
                # print("got imu packet")
        except:
            pkt = pkt

        try:
            str_buffer = read_socat(term)
            controller_sock.sendto(str_buffer.encode(),(JETSON_IP,CONTROLLER_PORT))
            print("sent")

            dec = json.loads(str_buffer)

            auto_mode_button = dec['BUTTON']['#04']
            stop_button = dec['BUTTON']['#12']
            back_button = dec['BUTTON']['#13']
            left_button = dec['BUTTON']['#14']
            right_button = dec['BUTTON']['#15']
            print(turn)

            if stop_button >= 0.5:
                turn = "no_turn"
                rpmR, rpmL = 0, 0

            elif (turn == "left_180_turn") or ((back_button > 0.5) and (left_button > 0.5)):
                rpmR, rpmL = turn_180(np.pi, 'left', pkt)
                print('left180')

            elif (turn == "right_180_turn") or ((back_button > 0.5) and (right_button > 0.5)):
                rpmR, rpmL = turn_180(np.pi, 'right', pkt)
                print('right180')

            elif turn == "left_turn" or left_button < old_left_button:
                rpmR, rpmL = turn_90(np.pi/2, 'left', pkt)
                print('left90')

            elif turn == "right_turn"or right_button < old_right_button:
                rpmR, rpmL = turn_90(np.pi/2, 'right', pkt)
                print('right90')

                """elif auto_mode_button >= 0.5:
                print("AUTO MODE")
                rpmR = float(rpmRs)
                rpmL = float(rpmLs)"""
            else:
                rpmL, rpmR = moter_rpm(dec['AXES']['#00'],dec['AXES']['#01']*-1)
            print(rpmR,rpmL)
            SendFloat(rpmR,rpmL)

            old_stop_button = stop_button
            old_left_button = left_button
            old_right_button = right_button

        except Exception as e:
            print(e)
            rpmL = rpmL * 0.5
            rpmR = rpmR * 0.5
            SendFloat(rpmR,rpmL)
            print('except2')
            #print("No control signal:"+str(rpmL),str(rpmR))
        
        str_buffer = ''
        #sys.stdout.flush()

