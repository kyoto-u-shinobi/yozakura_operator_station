#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pickle
import socket
import time

# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to server and send data
    sock.connect(("localhost", 9999))
    while True:
        sock.sendall("speeds\n")  # If it doesn't work, add "\n" too!

        # Receive data from the server
        received = sock.recv(1024)
        print received
        speed = pickle.loads(received)
        print "Received: {}".format(speed)

        time.sleep(1)

finally:
    sock.close()