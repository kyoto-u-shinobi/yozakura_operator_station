#!/usr/bin/env python
#  (C) 2015  Kyoto University Mechatronics Laboratory
# Released under the GNU General Public License, version 3


import logging
import pygame
import socket
import struct
import fcntl

from controller import Controller
from server import Server, Handler


def get_ip_address(interface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packed = struct.pack("256s", str.encode(interface))
    return socket.inet_ntoa(fcntl.ioctl(s.fileno(),
        0x8915,  # SIOCGIFADDR
        packed)[20:24])


def main():
    try:
        ip_address = get_ip_address("eth0")
    except OSError:
        # ip_address = get_ip_address("enp2s0")
        ip_address = get_ip_address("wlan0")
    server = Server((ip_address, 9999), Handler)

    logging.debug("Initializing controllers")
    try:
        stick_body = Controller(0, name="main")
        server.add_controller(stick_body)
    except pygame.error:
        logging.warning("No controller attached")

    try:
        logging.debug("Starting server")
        server.serve_forever()
    except (KeyboardInterrupt, SystemExit):
        raise
    finally:
        logging.info("Shutting down...")
        Controller.shutdown_all()
    logging.info("All done")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
