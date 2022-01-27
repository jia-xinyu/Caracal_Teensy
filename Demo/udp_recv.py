#!/usr/bin/python
import socket
UDP_IP = "192.168.137.138"   #Mac or Linux box's IP
#UDP_IP = "127.0.0.1"   #Loop
UDP_PORT = 8080

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
	data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	print("received message: %s" % data)
