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
import rospy
from std_msgs.msg import Float32


class SensorDataSender:
    def __init__(self):
        ros_topic_name = 'sensor_data_server'
        data_for_jointstate = ['body_front_pitch_deg', 'body_front_roll_deg',
                               'body_back_pitch_deg', 'body_back_roll_deg',
                               'flipper_left_deg', 'flipper_right_deg',
                               'starwheel_front_deg', 'starwheel_back_deg',
                               'armjack_triangle_topangle_deg',
                               'armbase_yaw_deg', 'armbase_pitch_deg']

        data_for_sensordisplay = ['current_motor_left', 'current_motor_right',
                                  'current_flipper_left', 'current_flipper_right',
                                  'voltage_motor_left', 'voltage_motor_right',
                                  'voltage_flipper_left', 'voltage_flipper_right',
                                  'current_battery', 'voltage_battery',
                                  'heat_sensor', 'co2_sensor']

        self.publish_dataname_list = data_for_jointstate + data_for_sensordisplay

        rospy.init_node(ros_topic_name, anonymous=True)

        self.publishers = {}
        self.published_data = {}
        for name in self.publish_dataname_list:
            self.publishers[name] = rospy.Publisher(name, Float32, queue_size=10)
            self.published_data[name] = 0

    def _send_published_data(self, destination, value):
        if value is not None:
            self.published_data[destination] = value
        else:
            self.published_data[destination] = -10000.0

    def set_data(self, pose, flipper_angles, arm_angles, current, voltage, heat, co2):
        print(pose)
        self._send_published_data('body_front_pitch_deg', pose[0][1])
        self._send_published_data('body_front_roll_deg', pose[0][0])
        self._send_published_data('body_back_pitch_deg', pose[1][1])
        self._send_published_data('body_back_roll_deg', pose[1][0])
        self._send_published_data('flipper_left_deg', flipper_angles[0])
        self._send_published_data('flipper_right_deg', flipper_angles[1])
        self._send_published_data('armjack_triangle_topangle_deg', arm_angles[0])
        self._send_published_data('armbase_yaw_deg', arm_angles[1])
        self._send_published_data('armbase_pitch_deg', arm_angles[2])

        self._send_published_data('current_motor_left', current[0])
        self._send_published_data('voltage_motor_left', voltage[0])
        self._send_published_data('current_motor_right', current[1])
        self._send_published_data('voltage_motor_right', voltage[1])
        self._send_published_data('current_flipper_left', current[2])
        self._send_published_data('voltage_flipper_left', voltage[2])
        self._send_published_data('current_flipper_right', current[3])
        self._send_published_data('voltage_flipper_right', voltage[3])
        self._send_published_data('current_battery', current[4])
        self._send_published_data('voltage_battery', voltage[4])

        self._send_published_data('heat_sensor', heat)
        self._send_published_data('co2_sensor', co2)

    def publish_data(self):
        for name in self.publish_dataname_list:
            self.publishers[name].publish(self.published_data[name])


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

        self._sensor_data_sender = SensorDataSender()

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
        self.wheels_single_stick = False
        self.reverse_mode = False
        self._sticks_timestamp = self._reverse_timestamp = time.time()

        # TODO(murata): Remove everything related to _sensors_client and the
        # try/finally block once you add your udp server.
        self._sensors_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sensors_client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sensors_client.bind(("", 9999))

        try:
            while True:
                try:
                    data = self.request.recv(64).decode().strip()
                except socket.timeout:
                    self._logger.warning("Lost connection to robot")
                    self._logger.info("Robot will shut down motors")
                    continue
                self._logger.debug('Received: "{}"'.format(data))

                if data == "":  # Client exited safely.
                    self._logger.info("Terminating client session")
                    break

                if data == "state":
                    state = self.server.controllers["main"].get_state()
                    reply = pickle.dumps(state)

                elif data == "inputs":
                    state = self.server.controllers["main"].get_state()
                    dpad, lstick, rstick, buttons = state.data
                    reply = pickle.dumps(((dpad.x, dpad.y),
                                          (lstick.x, lstick.y),
                                          (rstick.x, rstick.y),
                                          buttons.buttons))

                elif data == "speeds":
                    state = self.server.controllers["main"].get_state()
                    reply = pickle.dumps(self._get_needed_speeds(state))

                elif data.split()[0] == "echo":
                    reply = " ".join(data.split()[1:])

                elif data.split()[0] == "print":
                    reply = " ".join(data.split()[1:])
                    self._logger.info('Client says: "{}"'.format(reply))

                else:
                    reply = 'Unable to parse command: "{}"'.format(data)
                    self._logger.debug(reply)

                try:
                    self.request.sendall(str.encode(reply))
                except TypeError:  # Already bytecode
                    self.request.sendall(reply)

                # Receive sensor data
                raw_data, address = self._sensors_client.recvfrom(512)
                self._logger.debug("{}".format(pickle.loads(raw_data)))

                flipper_angles, current_sensor_data, pose_sensor_data = pickle.loads(raw_data)
                #print(pose_sensor_data)
                # flipper_angles = [0.0, 0.0]  # TODO
                arm_angles = [0.0, 0.0, 0.0]  # TODO
                heat = 0.0  # TODO
                co2 = 0.0  # TODO
                front_pose_sensor, rear_pose_sensor = pose_sensor_data
                front_pose_sensor_degrees = [np.rad2deg(i) if i is not None else None for i in front_pose_sensor]
                rear_pose_sensor_degrees = [np.rad2deg(i) if i is not None else None for i in rear_pose_sensor]
                self._sensor_data_sender.set_data([front_pose_sensor_degrees, rear_pose_sensor_degrees], flipper_angles,
                                                  arm_angles,
                                                  [reading[0] if reading is not None else None for reading in
                                                   current_sensor_data],
                                                  [reading[2] if reading is not None else None for reading in
                                                   current_sensor_data],
                                                  heat, co2)
                self._sensor_data_sender.publish_data()

                if rospy.is_shutdown():
                    break

        finally:
            self._sensors_client.close()

    def _get_needed_speeds(self, state):
        """
        Get required speeds based on controller state and system state.

        Inputs handled:
            - L1, L2 : Rotate left flipper.
            - R1, R2 : Rotate right flipper.
            - lstick : x- and y-axes control wheels in single-stick mode;
              y-axis controls left-side wheels in dual-stick mode.
            - rstick : y-axis controls right-side wheels in dual-stick
              mode.
            - L3 : Toggle the control mode between single and dual sticks.
            - R3 : Toggle reverse mode

        Parameters
        ----------
        state : State
            Represents the controller states.

        Returns
        -------
        float
            The speed inputs for each of the four motors, with values
            between -1 and 1. The four motors are:
                - Left motor
                - Right motor
                - Left flipper
                - Right flipper

        """
        # TODO(masasin): Handle select : Synchronize flipper positions.
        # TODO(masasin): Handle start : Move flippers to forward position.
        dpad, lstick, rstick, buttons = state.data

        if buttons.is_pressed("L3"):
            self._switch_control_mode()
        if buttons.is_pressed("R3"):
            self._engage_reverse_mode()

        if self.reverse_mode:
            # Wheels
            if self.wheels_single_stick:
                self._logger.debug("lx: {:9.7}  ".fromat(lstick.x) +
                                   "ly: {:9.7}".format(lstick.y))
                if abs(lstick.y) == 0:  # Rotate in place
                    lmotor = -lstick.x
                    rmotor = lstick.x
                else:
                    l_mult = (1 - lstick.x) / (1 + abs(lstick.x))
                    r_mult = (1 + lstick.x) / (1 + abs(lstick.x))
                    lmotor = lstick.y * l_mult
                    rmotor = lstick.y * r_mult
            else:
                self._logger.debug("ly: {:9.7}  ".fromat(lstick.y) +
                                   "ry: {:9.7}".format(rstick.y))
                lmotor = rstick.y
                rmotor = lstick.y

            # Flippers
            if buttons.all_pressed("L1", "L2"):
                rflipper = 0
            elif buttons.is_pressed("L1"):
                rflipper = 1
            elif buttons.is_pressed("L2"):
                rflipper = -1
            else:
                rflipper = 0

            if buttons.all_pressed("R1", "R2"):
                lflipper = 0
            elif buttons.is_pressed("R1"):
                lflipper = 1
            elif buttons.is_pressed("R2"):
                lflipper = -1
            else:
                lflipper = 0

        else:  # Forward mode
            # Wheels
            if self.wheels_single_stick:
                self._logger.debug("lx: {:9.7}  ".format(lstick.x) +
                                   "ly: {:9.7}".format(lstick.y))
                if abs(lstick.y) == 0:  # Rotate in place
                    lmotor = lstick.x
                    rmotor = -lstick.x
                else:
                    l_mult = (1 + lstick.x) / (1 + abs(lstick.x))
                    r_mult = (1 - lstick.x) / (1 + abs(lstick.x))
                    lmotor = -lstick.y * l_mult
                    rmotor = -lstick.y * r_mult
            else:
                self._logger.debug("ly: {:9.7}  ".format(lstick.y) +
                                   "ry: {:9.7}".format(rstick.y))
                lmotor = -lstick.y
                rmotor = -rstick.y

            # Flippers
            if buttons.all_pressed("L1", "L2"):
                lflipper = 0
            elif buttons.is_pressed("L1"):
                lflipper = 1
            elif buttons.is_pressed("L2"):
                lflipper = -1
            else:
                lflipper = 0

            if buttons.all_pressed("R1", "R2"):
                rflipper = 0
            elif buttons.is_pressed("R1"):
                rflipper = 1
            elif buttons.is_pressed("R2"):
                rflipper = -1
            else:
                rflipper = 0

        return lmotor, -rmotor, -lflipper, rflipper

    def _switch_control_mode(self):
        """
        Toggle the control mode between single and dual analog sticks.

        Ignores the toggle directive if the mode has been switched within the
        last second.

        """
        current_time = time.time()

        if current_time - self._sticks_timestamp >= 1:
            if self.wheels_single_stick:
                self.wheels_single_stick = False
                self._logger.info("Control mode switched: Use " +
                                  "lstick and rstick to control robot")
            else:
                self.wheels_single_stick = True
                self._logger.info("Control mode switched: Use " +
                                  "lstick to control robot")
            self._sticks_timestamp = current_time

    def _engage_reverse_mode(self):
        """
        Toggle the control mode between forward and reverse.

        In reverse mode, the regular inputs will cause the robot to move
        in reverse as if it were moving forward.

        Ignores the toggle directive if the mode has been switched within the
        last second.

        """
        current_time = time.time()

        if current_time - self._reverse_timestamp >= 1:
            if self.reverse_mode:
                self.reverse_mode = False
                self._logger.info("Reverse mode disabled!")
            else:
                self.reverse_mode = True
                self._logger.info("Reverse mode enabled!")
            self._reverse_timestamp = current_time


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
        self.controllers = {}

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

    def add_controller(self, controller):
        """
        Register a controller.

        Parameters
        ----------
        controller : Controller
            The controller to be registered.

        """
        self._logger.debug("Adding controller {}".format(controller))
        self.controllers[controller.name] = controller

    def remove_controller(self, controller):
        """Deregister a controller.

        Parameters
        ----------
        controller : Controller
            The controller to be deregistered.

        """
        self._logger.debug("Removing controller {}".format(controller))
        del self.controllers[controller.name]
