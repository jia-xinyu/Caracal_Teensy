#!/usr/bin/python

# Ethernet/UDP client to communicate to a remote Teensy server
# remote IP - 192.168.137.177
# Author: Jia, Xinyu
# Last modified: Jan 24, 2022

import sys
import socket
import copy as cp

UDP_IP = "192.168.137.177"  # Teensy IP
UDP_PORT = 8080

# 48 bytes = 24 uint16_t
class joint_command:
    def __init__(self):
        self.tau_a_des = [0.0, 0.0, 0.0]
        self.tau_b_des = [0.0, 0.0, 0.0]
        self.tau_c_des = [0.0, 0.0, 0.0]
        self.flags = [0, 0, 0]

# 108 bytes = 54 uint16_t
class joint_data:
    def __init__(self):
        self.q_a = [0.0, 0.0, 0.0]
        self.q_b = [0.0, 0.0, 0.0]
        self.q_c = [0.0, 0.0, 0.0]

        self.qd_a = [0.0, 0.0, 0.0]
        self.qd_b = [0.0, 0.0, 0.0]
        self.qd_c = [0.0, 0.0, 0.0]

        self.tau_a = [0.0, 0.0, 0.0]
        self.tau_b = [0.0, 0.0, 0.0]
        self.tau_c = [0.0, 0.0, 0.0]

        # int32_t flags = [0.0, 0.0, 0.0]
        # int32_t checksum

class low2high:
  _joint_data = joint_data 
  # _force_data = force_data 

class high2low:
  _joint_cmd = joint_command 

class udp_args:
  msgs_data = low2high 
  msgs_cmd = high2low 

# --------------------------------------------------

args_udp = udp_args
udp_tx = sys.getsizeof(args_udp.msgs_cmd)
# udp_rx = sys.getsizeof(args_udp.msgs_data)
print(udp_tx)

# init
MESSAGE = "Hello from PC client"
print("message: %s" % MESSAGE)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(bytes(MESSAGE), (UDP_IP, UDP_PORT))

while 1:
    # send Torque command
    args_udp.msgs_cmd._joint_cmd.tau_a_des = [1, 1, 1]
    args_udp.msgs_cmd._joint_cmd.tau_b_des = [1, 1, 1]
    args_udp.msgs_cmd._joint_cmd.tau_c_des = [1, 1, 1]

    # UDP send
    sock.sendto(bytes(args_udp.msgs_cmd), (UDP_IP, UDP_PORT))

    print(bytes(args_udp.msgs_cmd))

    # # UDP recv
    # data, addr = sock.recvfrom(udp_rx)
    # print("received message: %s" % data)

    # # print data
    # args_udp.msgs_data[0] = data[0]
    # if (data):
    #     print('%f %f %f %f %f %f %f %f %f', \
    #         args_udp.msgs_data._joint_data.q_a[0], \
    #         args_udp.msgs_data._joint_data.q_a[1], \
    #         args_udp.msgs_data._joint_data.q_a[2], \
    #         args_udp.msgs_data._joint_data.q_b[0], \
    #         args_udp.msgs_data._joint_data.q_b[1], \
    #         args_udp.msgs_data._joint_data.q_b[2], \
    #         args_udp.msgs_data._joint_data.q_c[0], \
    #         args_udp.msgs_data._joint_data.q_c[1], \
    #         args_udp.msgs_data._joint_data.q_c[2])
    
    if (input()):
        break
        
sock.close()
