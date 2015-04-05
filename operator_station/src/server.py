#!/usr/bin/env python
# -*- coding: utf-8 -*-

# (C) 2015  Kyoto University Mechatronics Laboratory
# Released under the GNU General Public License, version 3
"""
Server for Yozakura motor commands.

After connecting with the client, the server receives requests and responds
accordingly. Can read joystick input, and connect to multiple clients
simultaneously.

"""
import logging
import pickle
import socket
import SocketServer
import time

import numpy as np
from command_generator import CommandGenerator
from sensor_data_manager import SensorDataManager


class Handler(SocketServer.BaseRequestHandler):
    """
    A handler for connection requests.

    It gets called by the server automatically whenever a new client connects.

    Attributes
    ----------
    request : socket
        Handles communication with the client
    wheels_single_stick : bool
        Whether the wheels are controlled by only the left analog stick.
    reverse mode : bool
        Whether reverse mode is engaged. In reverse mode, the x- and y- inputs
        are both inverted.

    """

    def __init__(self, request, client_address, server):
        self._logger = logging.getLogger("{client_ip}_handler".format(
            client_ip=client_address[0]))
        self._logger.debug("New handler created")

        self.cmd_gen = CommandGenerator(self._logger, 'joy', 'main')
        self.cmd_gen.activate()
        self.sensor_mgr = SensorDataManager()

        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)

    def handle(self):
        """
        Handle the requests to the server.

        Once connected to the client, the handler loops and keeps listening for
        requests. This allows us to find out when the client is disconnected,
        and also allows for a much higher communication rate with the robot.

        Pickle is used on the server and client sides to transfer Python
        objects.

        Requests handled:
            - state : Reply with the state of the controller.
            - inputs : Reply with the raw input data from the state.
            - speeds : Perform calculations and send the required motor speed
              data.
            - echo : Reply with what the client has said.
            - print : ``echo``, and print to ``stdout``.

        """
        self._logger.info("Connected to client")
        self.request.settimeout(0.5)  # seconds
        self._sticks_timestamp = self._reverse_timestamp = time.time()

        self._sensors_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sensors_client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sensors_client.bind(("", 9999))
        try:
            self._loop()
        finally:
            self._sensors_client.close()
            raise SystemExit

    def _loop(self):
        """The main handler loop."""
        while True:
            try:
                data = self.request.recv(64).decode().strip()
            except socket.timeout:
                self._logger.warning("Lost connection to robot")
                self._logger.info("Robot will shut down motors")
                continue
            self._logger.debug('Received: "{data}"'.format(data=data))

            if data == "":  # Client exited safely.
                self._logger.info("Terminating client session")
                break

            elif data == "state":
                reply = pickle.dumps(self.cmd_gen.get_jsstate())

            elif data == "inputs":
                reply = pickle.dumps(self.cmd_gen.get_input())

            elif data == "speeds":
                reply = pickle.dumps(self.cmd_gen.get_speed_commands())

            elif data.split()[0] == "echo":
                reply = " ".join(data.split()[1:])

            elif data.split()[0] == "print":
                reply = " ".join(data.split()[1:])
                self._logger.info('Client says: "{reply}"'.format(reply=reply))

            else:
                reply = 'Unable to parse command: "{cmd}"'.format(cmd=data)
                self._logger.debug(reply)

            try:
                self.request.sendall(str.encode(reply))
            except TypeError:  # Already bytecode
                self.request.sendall(reply)

            # Receive sensor data
            raw_data, address = self._sensors_client.recvfrom(1024)
            try:
                adc_data, current_data, pose_data = pickle.loads(raw_data)
                self._log_sensor_data(adc_data, current_data, pose_data)

                # set data and publish
                self.sensor_mgr.set_data(adc_data[0:2], current_data, pose_data)
                self.sensor_mgr.publish_data()
            except (AttributeError, EOFError, IndexError, TypeError):
                self._logger.debug("No or bad data received from robot")

    def _log_sensor_data(self, adc_data, current_data, pose_data):
        """
        Log sensor data to debug.
        Parameters
        ----------
        adc_data : 2-list of floats
            ADC data containing flipper positions, in radians.
        current_data : 3-list of 3-list of floats
            Current sensor data containing current, power, and voltage values.
        pose_data : 2-list of 3-list of floats
            Pose data containing yaw, pitch, and roll values.
        """
        lwheel, rwheel, lflip, rflip, battery = current_data
        front, rear = np.rad2deg(pose_data)

        self._logger.debug("lflipper: {lf:6.3f}  rflipper: {rf:6.3f}"
                           .format(lf=adc_data[0], rf=adc_data[1]))
        self._logger.debug("lwheel_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V"
                           .format(i=lwheel[0], p=lwheel[1], v=lwheel[2]))
        self._logger.debug("rwheel_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V"
                           .format(i=rwheel[0], p=rwheel[1], v=rwheel[2]))
        self._logger.debug("lflip_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V"
                           .format(i=lflip[0], p=lflip[1], v=lflip[2]))
        self._logger.debug("rflip_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V"
                           .format(i=rflip[0], p=rflip[1], v=rflip[2]))
        self._logger.debug("batt_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V"
                           .format(i=battery[0], p=battery[1], v=battery[2]))
        self._logger.debug("front r: {r:6.3f}  p: {p:6.3f}  y: {y:6.3f}"
                           .format(r=front[0], p=front[1], y=front[2]))
        self._logger.debug("rear r: {r:6.3f}  p: {p:6.3f}  y: {y:6.3f}"
                           .format(r=rear[0], p=rear[1], y=rear[2]))
        self._logger.debug(20 * "=")


class Server(SocketServer.ForkingMixIn, SocketServer.TCPServer):
    """
    A TCP Server.

    Parameters
    ----------
    server_address : 2-tuple of (str, int)
        The address at which the server is listening. The elements are the
        server address and the port number respectively.
    handler_class : Handler
        The request handler. Each new request generates a separate process
        running that handler.

    Attributes
    ----------
    controllers : dict
        Contains all registered motors.

        **Dictionary format :** {name (str): controller (Controller)}

    Examples
    --------
    >>> server = Server(("192.168.11.1", 22), Handler)
    >>> server.serve_forever()

    """
    allow_reuse_address = True  # Can resume immediately after shutdown

    def __init__(self, server_address, handler_class):
        SocketServer.TCPServer.__init__(self, server_address, handler_class)

        self._logger = logging.getLogger("{}_server".format(server_address[0]))
        self._logger.debug("Creating server")

        self._logger.info("Listening to port {}".format(server_address[1]))

    def serve_forever(self, poll_interval=0.5):
        """
        Handle requests until an explicit ``shutdown()`` request.

        Parameters
        ----------
        poll_interval : float, optional
            The polling interval, in seconds.

        """
        self._logger.info("Server started")
        try:
            SocketServer.TCPServer.serve_forever(self, poll_interval)
        except (KeyboardInterrupt, SystemExit):
            pass

