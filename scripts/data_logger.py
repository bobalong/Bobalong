#!/usr/bin/python
#
# data_logger.py
#

import serial, sys, time
from socket import *

# variables
udp_port = 8888

# setup udp socket
udp_sock = socket(AF_INET, SOCK_DGRAM)
udp_sock.bind(('<broadcast>', 0))
udp_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

# setup yun serial
yun_serial = serial.Serial('/dev/ttyATH0', 115200);

while True:
	# blocks until we have read a whole line
	data = yun_serial.readline();
	print data
	# log the data and broadcast it over the yun's wifi as a UDP packet
	udp_sock.sendto(data, ('<broadcast>', udp_port))
	with open('log', 'a') as log_file:
		log_file.write(str(data))