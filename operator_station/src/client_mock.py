#!/usr/bin/env python
# -*- coding: utf-8 -*-


import socket, pickle


class Client(object):
    def __init__(self, client_address, server_address):
        self.request = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.request.connect(server_address)

        self.request.settimeout(0.5)  # seconds

        self.server_address = server_address
        self._sensors_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motors = {}
        self.serials = {}
        self.current_sensors = {}
        self.imus = {}


    def run(self):
        while True:
            print('client running')
            try:
                self.request.send(str.encode("speeds"))  # Request speeds.
                result = self.request.recv(64)  # Receive speed data.
                if not result:
                    continue
            except socket.timeout:
                continue

            try:
                lwheel, rwheel, lflipper, rflipper = pickle.loads(result)
                print( lwheel, rwheel, lflipper, rflipper)
            except EOFError:
                continue
            adc_data = [1.0, 2.0]
            current_data = [[11.0, 12.0, 13.0],  # lwheel
                            [21.0, 22.0, 23.0],  # rwheel
                            [31.0, 32.0, 33.0],  # lflip
                            [41.0, 42.0, 43.0],  # rflip
                            [51.0, 52.0, 53.0]]  # battery
            imu_data = [[101.0, 102.0, 103.0],  # front
                        [201.0, 202.0, 203.0]]  # rear

            self._sensors_server.sendto(pickle.dumps((adc_data, current_data, imu_data),
                                                     protocol=2),
                                        self.server_address)

    def shutdown(self):
        self.request.close()


# -------------------------------------------------------------------------
if __name__ == "__main__":
    client_address = "localhost"
    opstn_address = "localhost"
    client = Client(client_address, (opstn_address, 9999))
    try:
        client.run()
    finally:
        client.shutdown()
