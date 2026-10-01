#!/usr/bin/env python3
"""Module to control OnRobot's grippers.

Adapted from:
https://github.com/ian-chuang/OnRobot-RG2FT-ROS/tree/4d13e98dfa870c6a670f24120ff6c3998229c2b6
https://github.com/fraunhoferhhi/Ros2-OnRobot-RG2-FT/blob/main/onrobot_rg_control/modbusTcp/comModbusTcp.py
https://github.com/tonydle/OnRobot_ROS2_Driver/blob/main/include/onrobot_driver/RG.hpp
"""

import sys
import threading
import time
import struct
from enum import Enum
from typing import OrderedDict, Tuple, Union
try:
	from pymodbus.client.sync import ModbusTCPClient as ModbusClient
except:
	from pymodbus.client import ModbusTCPClient as ModbusClient

class OnRobotGripper:
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusClient):
                instance of ModbusClient to establish modbus connection
            lock (threading.Lock):
                instance of the threading.Lock to achieve exclusive control

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
            getStatus: Sends a request to read and returns the gripper status.
    """
    # Registers
    DEVICE_ID = 65
    REG = {
        'SetZero':                      0,
        'TargetForce':                  2,
        'TargetWidth':                  3,
        'Control':                      4,
        'ProximityOffsetL':             5,
        'ProximityOffsetR':             6,
        'StatusL':                      257,
        'FxL':                          259,
        'FyL':                          260,
        'FzL':                          261,
        'TxL':                          262,
        'TyL':                          263,
        'TzL':                          264,
        'StatusR':                      266,
        'FxR':                          268,
        'FyR':                          269,
        'FzR':                          270,
        'TxR':                          271,
        'TyR':                          272,
        'TzR':                          273,
        'ProximityStatusL':             274,
        'ProximityValueL':              275,
        'ProximityStatusR':             277,
        'ProximityValueR':              278,
        'ActualWidth':                  280,
        'Busy':                         281,
        'GripDetected':                 282,
        'IsZero':                       283,
    }

    # Command values
    CMD = {
        'Grip':                         1,
        'Stop':                         8,
        'GripWithOffset':               16,
    }

    def __init__(self):
        self.client = None
        self.lock = threading.Lock()
        self._min_position = 0
        self._max_position = 1000
        self._min_speed = 0 # No speed control
        self._max_speed = 0 # No speed control
        self._min_force = 0
        self._max_force = 400

    def connect(self, hostname: str, port: int, socket_timeout: float = 10.0) -> None:
        """ Connects to the client device (gripper).

            Args:
                hostname (str): IP address (e.g. '192.168.1.1')
                port (str): port number (e.g. '502')
        """
        self.client = ModbusClient(
            hostname,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=socket_timeout,)
        if not self.client.connect():
            print("Unable to connect to {}:{}".format(hostname, port))

    def disconnect(self) -> None:
        """ Closes connection. """
        self.client.close()

    def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]) -> bool:
        """Sends the appropriate command via socket to set the value of n variables, and waits for its 'ack' response.

        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        assert self.client is not None
        success = True
        # atomic commands send/rcv
        with self.lock:
            for variable, value in var_dict.items():
                address = self.REG[variable]
                write_result = self.client.write_register(
                    address=address, value=value, unit=self.DEVICE_ID)
                if write_result.isError(): success = False
        return success

    def _set_var(self, variable: str, value: Union[int, float]) -> bool:
        """Sends the appropriate command via socket to set the value of a variable, and waits for its 'ack' response.

        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable: str) -> int:
        """Sends the appropriate command to retrieve the value of a variable from the gripper, blocking until the response is received or the socket times out.

        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        assert self.client is not None
        value = None
        # atomic commands send/rcv
        with self.lock:
            value = self.client.read_holding_registers(
                address=self.REG[variable], count=1, unit=self.DEVICE_ID).registers
        return value
    
    def _reset(self) -> None:
        """Reset the gripper.
        """
        self._set_var('SetZero', 1)
        self._set_var('Control', self.CMD['Stop'])
        while not self._get_var('IsZero') == 1 \
                or not self._get_var('StatusL') == 0 \
                or not self._get_var('StatusR') == 0 \
                or not self._get_var('ProximityStatusL') == 0 \
                or not self._get_var('ProximityStatusR') == 0:
            self._set_var('SetZero', 1)
            self._set_var('Control', self.CMD['Stop'])
        time.sleep(0.5)
        
    def activate(self, auto_calibrate: bool = True):
        """Resets the activation flag in the gripper, and sets it back to one, clearing previous fault flags.
        """
        if not self.is_active():
            self._reset()
            while not self._get_var('IsZero') == 1 or not self._get_var('Busy') == 0:
                time.sleep(0.01)

            self._set_var('SetZero', 0)
            time.sleep(1.0)
            while not self._get_var('IsZero') == 0 \
                or not self._get_var('StatusL') == 0 \
                or not self._get_var('StatusR') == 0 \
                or not self._get_var('ProximityStatusL') == 0 \
                or not self._get_var('ProximityStatusR') == 0:
                time.sleep(0.01)

        # auto-calibrate position range if desired
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        """Returns whether the gripper is active."""
        status = self._get_var('IsZero') == 0 \
                and self._get_var('StatusL') == 0 \
                and self._get_var('StatusR') == 0 \
                and self._get_var('ProximityStatusL') == 0 \
                and self._get_var('ProximityStatusR') == 0
        return status

    def get_min_position(self) -> int:
        """Returns the minimum position the gripper can reach (open position)."""
        return self._min_position

    def get_max_position(self) -> int:
        """Returns the maximum position the gripper can reach (closed position)."""
        return self._max_position

    def get_open_position(self) -> int:
        """Returns what is considered the open position for gripper (maximum position value)."""
        return self.get_max_position()

    def get_closed_position(self) -> int:
        """Returns what is considered the closed position for gripper (minimum position value)."""
        return self.get_min_position()

    def is_open(self):
        """Returns whether the current position is considered as being fully open."""
        return self.get_current_position() >= self.get_open_position()

    def is_closed(self):
        """Returns whether the current position is considered as being fully closed."""
        return self.get_current_position() <= self.get_closed_position()

    def get_current_position(self) -> int:
        """Returns the current position as returned by the physical hardware."""
        return self._get_var('ActualWidth')

    def auto_calibrate(self, log: bool = True) -> None:
        """Attempts to calibrate the open and closed positions, by slowly closing and opening the gripper.

        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if self._get_var('GripDetected'):
            raise RuntimeError(f"Calibration failed opening to start: {str(status)}")

        # try to close as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(
            self.get_closed_position(), 64, 1
        )
        if self._get_var('GripDetected'):
            raise RuntimeError(
                f"Calibration failed because of an object: {str(status)}"
            )
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if self._get_var('GripDetected'):
            raise RuntimeError(
                f"Calibration failed because of an object: {str(status)}"
            )
        assert position >= self._min_position
        self._min_position = position

        if log:
            print(
                f"Gripper auto-calibrated to [{self.get_min_position()}, {self.get_max_position()}]"
            )

    def move(self, position: int, speed: int=0, force: int=0) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """
        position = int(position)
        speed = int(speed)
        force = int(force)

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict(
            [
                ('TargetWidth', clip_pos),
              # ('TargetSpeed', clip_spe), # RG2-FT gripper lacks speed control
                ('TargetForce', clip_for),
                ('Control',            1),
            ]
        )
        succ = self._set_vars(var_dict)
        time.sleep(0.008)  # need to wait (dont know why)
        return succ, clip_pos

    def move_and_wait_for_pos(
        self, position: int, speed: int=0, force: int=0
    ) -> Tuple[int, int]:  # noqa
        """Sends commands to start moving towards the given position, with the specified speed and force, and then waits for the move to complete.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with an integer representing the last position returned by the gripper after it notified
        that the move had completed, a status indicating how the move ended (see ObjectStatus enum for details). Note
        that it is possible that the position was not reached, if an object was detected during motion.
        """
        position = int(position)
        speed = int(speed)
        force = int(force)

        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while not self._get_var('Busy'):
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self._get_var('GripDetected')
        while (
            self._get_var('Busy')
        ):
            cur_obj = self._get_var('GripDetected')

        # report the actual position and the object status
        final_pos = self._get_var('ActualWidth')
        final_obj = cur_obj
        return final_pos, final_obj

    def restartPowerCycle(self) -> None:
        """ Restarts the power cycle of Compute Box.

            Necessary is Safety Switch of the grippers are pressed
            Writing 2 to this field powers the tool off
            for a short amount of time and then powers them back
        """

        message = 2
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            self.client.write_registers(
                address=0, values=message, unit=restart_address)


def main():
    # test open and closing the gripper
    gripper = OnRobotGripper()
    gripper.connect(hostname="192.168.1.1", port=502)
    gripper.activate()
    print(gripper.get_current_position())
    gripper.move_and_wait_for_pos(20, 0, 1)
    time.sleep(0.2)
    print(gripper.get_current_position())
    gripper.move_and_wait_for_pos(980, 0, 1)
    time.sleep(0.2)
    print(gripper.get_current_position())
    gripper.move_and_wait_for_pos(20, 0, 1)
    time.sleep(0.2)
    print(gripper.get_current_position())
    gripper.disconnect()


if __name__ == "__main__":
    main()
