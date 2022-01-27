#!/usr/bin/python
import socket
UDP_IP = "192.168.137.177"   #Mac or Linux box's IP
#UDP_IP = "127.0.0.1"   #Loop
UDP_PORT = 8080

MESSAGE = "Hello from server"
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
sock.sendto(bytes(MESSAGE), (UDP_IP, UDP_PORT))
sock.close()

