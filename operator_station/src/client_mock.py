#!/usr/bin/env python
# -*- coding: utf-8 -*-


import socket, pickle, time, math


class Client(object):
    """
    A client mock for the test. See main to know how to use
    """

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
        """
        Send request 'speeds' and receive speed commands(base_vels, arm_vels)
        And send dummy sensor data
        """
        while True:
            print('client running')
            try:
                self.request.send(str.encode("commands"))  # Request commands.
                result = self.request.recv(128)  # Receive speed data.
                if not result:
                    continue
            except socket.timeout:
                continue

            try:
                commands, arms = pickle.loads(result)
                print(commands, arms)
            except EOFError:
                continue

            adc_data = [0.33780, 0.61080]
            current_data = [[11.0, 12.0],  # lwheel
                            [21.0, 22.0],  # rwheel
                            [31.0, 32.0],  # lflip
                            [41.0, 42.0]]  # rflip
            imu_data = [[math.radians(101.0), math.radians(102.0), math.radians(103.0)],  # front
                        [math.radians(201.0), math.radians(202.0), math.radians(203.0)]]  # rear

            arm_data = [[287.0, 160.0, 0.0],
                        [11.0, 12.0, 13.0],
                        [range(16), range(16)],
                        110.0]
            self._sensors_server.sendto(pickle.dumps((adc_data, current_data, imu_data, arm_data),
                                                     protocol=2),
                                        self.server_address)

            time.sleep(0.2)  # to control this loop rate

    def shutdown(self):
        self.request.close()

# --------------------------------------------------------------
if __name__ == "__main__":
    client_address = "localhost"
    opstn_address = "localhost"
    client = Client(client_address, (opstn_address, 9999))
    try:
        client.run()
    finally:
        client.shutdown()
