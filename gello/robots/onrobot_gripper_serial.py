#!/usr/bin/env python3
"""Module to control OnRobot's grippers.

Taken from https://github.com/ian-chuang/OnRobot-RG2FT-ROS/tree/4d13e98dfa870c6a670f24120ff6c3998229c2b6
"""

import sys
import threading
import time
import rospy
import struct
from enum import Enum
from typing import OrderedDict, Tuple, Union
try:
	from pymodbus.client.sync import ModbusSerialClient
except:
	from pymodbus.client import ModbusSerialClient

RG2FT_MIN_WIDTH = 1
RG2FT_MAX_WIDTH = 1000
RG2FT_MIN_FORCE = 0
RG2FT_MAX_FORCE = 400
RG2FT_DEVICE_ADDRESS = 65

def s16(val):
    return struct.unpack('h', struct.pack('H', val))[0]

def u16(val):
    return struct.unpack('H', struct.pack('h', val))[0]

class RG2FTCommand():
    TargetForce=0 # The target force to be reached when gripping and holding a workpiece. (1/10 N)
    TargetWidth=0 # The target width between the finger to be moved to and maintained. (1/10 mm)
    Control=0 # The control field is used to start and stop gripper motion. (0x0001 - grip, 0x0000 - stop)

class RG2FTState():
    StatusL = 0 # Reads low (0x0000) when there is no error with the left finger sensor.
    FxL = 0 # Left finger sensor's force value along the X axis (in the sensor coordinate system) in 1/10N.
    FyL = 0 # Left finger sensor's force value along the Y axis (in the sensor coordinate system) in 1/10N.
    FzL = 0 # Left finger sensor's force value along the Z axis (in the sensor coordinate system) in 1/10N.
    TxL = 0 # Left finger sensor's torque value about the X axis (in the sensor coordinate system) in 1/100 Nm.
    TyL = 0 # Left finger sensor's torque value about the Y axis (in the sensor coordinate system) in 1/100 Nm.
    TzL = 0 # Left finger sensor's torque value about the Z axis (in the sensor coordinate system) in 1/100 Nm.
    StatusR = 0 # Same as the left above.
    FxR = 0 # Same as the left above.
    FyR = 0 # Same as the left above.
    FzR = 0 # Same as the left above.
    TxR = 0 # Same as the left above.
    TyR = 0 # Same as the left above.
    TzR = 0 # Same as the left above.
    ProximityStatusL = 0 # Reads low (0x0000) when there is no error with the left proximity sensor.
    ProximityValueL = 0 # Reads the current distance from the left proximity sensor in 1/10 mm.
    ProximityStatusR = 0 # Same as the left above.
    ProximityValueR = 0 # Same as the left above.
    ActualGripperWidth = 0 # Indicates the current width between the gripper fingers in 1/10 millimeters.
    GripperBusy = 0 # High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
    GripDetected = 0 # High (1) when an internal or external grip is detected.

class OnRobotSerialClient:
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusTcpClient):
                instance of ModbusTcpClient to establish modbus connection
            lock (threading.Lock):
                instance of the threading.Lock to achieve exclusive control

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
            getStatus: Sends a request to read and returns the gripper status.
    """

    def __init__(self):
        self.client = None
        self.lock = threading.Lock()

    def connect(self, device, changer_addr=65):
        """ Connects to the client device (gripper).

            Args:
                ip (str): IP address (e.g. '192.168.1.1')
                port (str): port number (e.g. '502')
                changer_addr (int): quick tool changer address
        """
        self.client = ModbusSerialClient(method='rtu',port=device,stopbits=1, bytesize=8, baudrate=115200, timeout=0.2)
        if not self.client.connect():
            print("Unable to connect to {}".format(device))
            return False
        self.changer_addr = changer_addr

    def disconnect(self):
        """ Closes connection. """
        self.client.close()

    def write(self, address, message):
        """ Sends a command to the Gripper.

            Args:
                message (list[int]): message to be sent
        """

        # Sending a command to the device (address 0 ~ 2)
        if len(message) > 0:
            with self.lock:
                self.client.write_registers(
                    address=address, values=message, unit=self.changer_addr)

    def read(self, address, count):
        """ Sends a request to read and returns the gripper status. """
        # Getting status from the device (address 257 ~ 282)
        print(self.client.connected)
        read = None
        with self.lock:
            while (read is None) or (read.isError()):
                read = self.client.read_holding_registers(
                    address=address, count=count, unit=self.changer_addr)
            response = read.registers

        # Output the result
        return response

    def restartPowerCycle(self):
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

class OnRobotRG2FT:

    def __init__(self, device):
        self.client = OnRobotSerialClient()
        self.client.connect(device, RG2FT_DEVICE_ADDRESS)
        time.sleep(1)

    def verifyCommand(self, cmd):
        cmd.TargetForce = max(RG2FT_MIN_FORCE, cmd.TargetForce)
        cmd.TargetForce = min(RG2FT_MAX_FORCE, cmd.TargetForce)
        
        cmd.TargetWidth = max(RG2FT_MIN_WIDTH, cmd.TargetWidth)
        cmd.TargetWidth = min(RG2FT_MAX_WIDTH, cmd.TargetWidth)

        return cmd
        

    def writeCommand(self, cmd: RG2FTCommand):
        if cmd.TargetWidth < RG2FT_MIN_WIDTH or cmd.TargetWidth > RG2FT_MAX_WIDTH:
           rospy.logerr("[OnRobotRG2FT] Target width out of range")
           return 
           
        if cmd.TargetForce < RG2FT_MIN_FORCE or cmd.TargetForce > RG2FT_MAX_FORCE:
           rospy.logerr("[OnRobotRG2FT] Target force out of range")
           return 

        if cmd.Control not in [0, 1]:
            rospy.logerr("[OnRobotRG2FT] Control is not 0 or 1")
            return 

        message = [0] * 3

        message[0] = cmd.TargetForce
        message[1] = cmd.TargetWidth
        message[2] = cmd.Control

        self.client.write(address=2, message=message)

    def readState(self) -> RG2FTState:
        resp = self.client.read(address=257, count=26)

        msg = RG2FTState()

        msg.StatusL = resp[0] # Reads low (0x0000) when there is no error with the left finger sensor.
        msg.FxL = s16(resp[2]) # Left finger sensor's force value along the X axis (in the sensor coordinate system) in 1/10N.
        msg.FyL = s16(resp[3]) # Left finger sensor's force value along the Y axis (in the sensor coordinate system) in 1/10N.
        msg.FzL = s16(resp[4]) # Left finger sensor's force value along the Z axis (in the sensor coordinate system) in 1/10N.
        msg.TxL = s16(resp[5]) # Left finger sensor's torque value about the X axis (in the sensor coordinate system) in 1/100 Nm.
        msg.TyL = s16(resp[6]) # Left finger sensor's torque value about the Y axis (in the sensor coordinate system) in 1/100 Nm.
        msg.TzL = s16(resp[7]) # Left finger sensor's torque value about the Z axis (in the sensor coordinate system) in 1/100 Nm.
        msg.StatusR = resp[9] # Same as the left above.
        msg.FxR = s16(resp[11]) # Same as the left above.
        msg.FyR = s16(resp[12]) # Same as the left above.
        msg.FzR = s16(resp[13]) # Same as the left above.
        msg.TxR = s16(resp[14]) # Same as the left above.
        msg.TyR = s16(resp[15]) # Same as the left above.
        msg.TzR = s16(resp[16]) # Same as the left above.
        msg.ProximityStatusL = resp[17] # Reads low (0x0000) when there is no error with the left proximity sensor.
        msg.ProximityValueL = s16(resp[18]) # Reads the current distance from the left proximity sensor in 1/10 mm.
        msg.ProximityStatusR = resp[20] # Same as the left above.
        msg.ProximityValueR = s16(resp[21]) # Same as the left above.
        msg.ActualGripperWidth = s16(resp[23]) # Indicates the current width between the gripper fingers in 1/10 millimeters.
        msg.GripperBusy = resp[24] # High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
        msg.GripDetected = resp[25] # High (1) when an internal or external grip is detected.

        return msg

    def setProximityOffsets(self, left_offset, right_offset):
        message = [0] * 2

        message[0] = left_offset # This field sets the offset of the left proximity sensor that is subtracted from the raw signal. It must be provided in 1/10 millimeters. 
        message[1] = right_offset # Same as the left above.

        print(message)

        self.client.write(address=5, message=message)

    def zeroForceTorque(self, val):
        message = [0] * 1

        message[0] = val # Zero the force and torque values to cancel any offset.

        self.client.write(address=0, message=message)

    def restartPowerCycle(self):
        self.client.restartPowerCycle()

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """
        cmd = RG2FTCommand()
        cmd.TargetForce = force
        cmd.TargetWidth = position
        cmd.Control = 0x0001
        
        cmd = self.verifyCommand(cmd)
        self.client.writeCommand(cmd)
        
    def get_current_position(self) -> int:
        """Returns the current position as returned by the physical hardware."""
        # return self._get_var(self.POS)
        status = self.readState()
        return status.ActualGripperWidth


def main():
    ip = "192.168.1.1"
    port = "502"
    # test open and closing the gripper
    gripper = OnRobotRG2FT(ip, port)
    # gripper.activate()
    print(gripper.get_current_position())
    gripper.writeCommand(RG2FTCommand(50, 1000, 0x0001))
    time.sleep(1.0)
    print(gripper.get_current_position())
    gripper.writeCommand(RG2FTCommand(50, 0, 0x0001))
    time.sleep(1.0)
    print(gripper.get_current_position())
    gripper.writeCommand(RG2FTCommand(50, 1000, 0x0001))
    time.sleep(1.0)
    gripper.writeCommand(RG2FTCommand(0, 1000, 0x0000))
    gripper.client.disconnect()


if __name__ == "__main__":
    main()
