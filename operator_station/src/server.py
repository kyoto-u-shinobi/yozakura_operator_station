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
import select
import socket
import SocketServer
import time
from exceptions import *

import numpy as np

import rospy
from yozakura_msgs.msg import YozakuraCommand
from std_msgs.msg import String
from sensor_data_manager import SensorDataManager

# remap-able
DEFAULT_COMMAND_TOPICNAME = "yozakura_command"


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

    class Command:
        def __init__(self):
            self.lwheel, self.rwheel = 0.0, 0.0
            self.lflipper, self.rflipper = 0.0, 0.0
            self.arm_linear, self.arm_pitch, self.arm_yaw = 0.0, 0.0, 0.0

        def set_command(self, yozakura_command):
            '''
            command mapping function from YozakuraCommand
            :param yozakura_command:
            :return: commands for the RasPi
            '''
            if yozakura_command.base_vel_input_mode == 1:
                self.lwheel = yozakura_command.wheel_left_vel
                self.rwheel = yozakura_command.wheel_right_vel
            elif yozakura_command.base_vel_input_mode == 2:
                self.lwheel = yozakura_command.base_vel.linear.x
                self.rwheel = yozakura_command.base_vel.angular.z
            else:
                self.lwheel = 0.0
                self.rwheel = 0.0

            #print(self.lwheel)

            self.lflipper = yozakura_command.flipper_left_vel.angle
            self.rflipper = yozakura_command.flipper_right_vel.angle

            self.arm_linear = yozakura_command.arm_vel.top_angle
            self.arm_pitch = yozakura_command.arm_vel.pitch
            self.arm_yaw = yozakura_command.arm_vel.yaw

        def get_command(self):
            return self.lwheel, self.rwheel, self.lflipper, self.rflipper


    def __init__(self, request, client_address, server):

        rospy.init_node('operator_station', anonymous=True)
        self._subscriber = rospy.Subscriber(DEFAULT_COMMAND_TOPICNAME, YozakuraCommand, self._command_callback)

        self._logger = logging.getLogger("{client_ip}_handler".format(client_ip=client_address[0]))
        self._logger.debug("New handler created")

        self._command = self.Command()
        self._sensor_mgr = SensorDataManager()
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)


    def _command_callback(self, ycommand):
        self._command.set_command(ycommand)


    def handle(self):
        """
        Handle the requests to the server.

        Once connected to the client, the handler loops and keeps listening for
        requests. This allows us to find out when the client is disconnected,
        and also allows for a much higher communication rate with the robot.

        Pickle is used on the server and client sides to transfer Python
        objects.

        Requests handled:
            - speeds : Perform calculations and send the required motor speed data.
            - echo : Reply with what the client has said.
            - print : ``echo``, and print to ``stdout``.

        """
        self._logger.info("Connected to client")
        self.request.settimeout(0.5)  # seconds

        self._sensors_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sensors_client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        self._sensors_client.bind(("", 9999))

        try:
            self._loop()
        finally:
            self._subscriber.unregister()
            self._sensors_client.close()
            #raise SystemExit

    def _loop(self):
        """The main handler loop."""

        while not rospy.is_shutdown():
            try:
                data = self.request.recv(64).decode().strip()
            except socket.error:
                print("Bad data")
                continue
            except socket.timeout:
                self._logger.warning("Lost connection to robot")
                self._logger.info("Robot will shut down motors")
                print("Lost connection")
                continue

            self._logger.debug('Received: "{data}"'.format(data=data))

            if data == "":  # Client exited safely.
                self._logger.info("Terminating client session")
                print("Terminating client session")
                break

            elif data == "speeds":
                reply = pickle.dumps(self._command.get_command())

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
            raw_data = self._udp_receive(size=1024)

            try:
                adc_data, current_data, pose_data = pickle.loads(raw_data)
                self._log_sensor_data(adc_data, current_data, pose_data)
            
                # set data and publish
                self._sensor_mgr.set_data(adc_data[-2:], current_data, pose_data)
                self._sensor_mgr.publish_data()
            
            except (AttributeError, EOFError, IndexError, TypeError):
                #self._logger.debug("No or bad data received from robot")
                print("No or bad data received from robot")

    def _udp_get_latest(self, size=1, n_bytes=1):
        """
        Get the latest input.

        This function automatically empties the given socket queue.

        Parameters
        ----------
        size : int, optional
            The number of bytes to read at a time.
        n_bytes : int, optional
            The number of bytes to return.

        Returns
        -------
        bytes
            The last received message, or None if the socket was not ready.

        """
        data = []
        input_ready, o, e = select.select([self._sensors_client], [], [], 0)  # Check ready.
        while input_ready:
            data.append(input_ready[0].recv(size))  # Read once.
            input_ready, o, e = select.select([self._sensors_client], [], [], 0)  # Check ready.

        if not data:
            return None
        elif n_bytes == 1:
            return data[-1]
        else:
            return data[-n_bytes:]

    def _udp_receive(self, size=32):
        """
        Receive UDP data without blocking.

        Parameters
        ----------
        size : int
            The number of bytes to read at a time.

        Returns
        -------
        recv_msg : bytes
            The received message, or None if the buffer was empty.
        """
        recv_msg = None

        try:
            recv_msg = self._udp_get_latest(size)
        except BlockingIOError:
            pass

        return recv_msg

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
        adc_data = [i if i is not None else 0 for i in adc_data]
        lwheel = [i if i is not None else 0 for i in lwheel]
        rwheel = [i if i is not None else 0 for i in rwheel]
        lflip = [i if i is not None else 0 for i in lflip]
        rflip = [i if i is not None else 0 for i in rflip]
        battery = [i if i is not None else 0 for i in battery]

        try:
            front, rear = np.rad2deg(pose_data)
        except AttributeError:
            front = rear = [0, 0, 0]

        #print("lflipper: {lf:6.3f}  rflipper: {rf:6.3f}".format(lf=adc_data[0], rf=adc_data[1]))
        #print("lwheel_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V".format(i=lwheel[0], p=lwheel[1], v=lwheel[2]))
        #print("rwheel_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V".format(i=rwheel[0], p=rwheel[1], v=rwheel[2]))
        #print("lflip_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V".format(i=lflip[0], p=lflip[1], v=lflip[2]))
        #print("rflip_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V".format(i=rflip[0], p=rflip[1], v=rflip[2]))
        #print("batt_current: {i:6.3f} A  {p:6.3f} W  {v:6.3f} V".format(i=battery[0], p=battery[1], v=battery[2]))
        #print("front r: {r:6.3f}  p: {p:6.3f}  y: {y:6.3f}".format(r=front[0], p=front[1], y=front[2]))
        #print("rear r: {r:6.3f}  p: {p:6.3f}  y: {y:6.3f}".format(r=rear[0], p=rear[1], y=rear[2]))
        #print(20 * "=")


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
        print("Listening to port {}".format(server_address[1]))

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

