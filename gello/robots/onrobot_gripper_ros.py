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
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
from geometry_msgs.msg import Wrench


RG2FT_MIN_WIDTH = 0
RG2FT_MAX_WIDTH = 1000
RG2FT_MIN_FORCE = 0
RG2FT_MAX_FORCE = 400
RG2FT_DEVICE_ADDRESS = 65

class OnRobotRG2FTROS:

    def __init__(self):
        rospy.init_node('onrobot_rg2ft_gello', anonymous=True)

        self.cmd_pub = rospy.Publisher('command', RG2FTCommand, queue_size=1)

        self.state_pub = rospy.Subscriber('state', RG2FTState, self.stateCallback)
        self.left_wrench_pub = rospy.Subscriber('left_wrench', Wrench, self.leftWrenchCallback)
        self.right_wrench_pub = rospy.Subscriber('right_wrench', Wrench, self.rightWrenchCallback)

        self.state = None

        time.sleep(2)

    def stateCallback(self, message):
        self.state = message

    def leftWrenchCallback(self, message):
        self.left_wrench = message

    def rightWrenchCallback(self, message):
        self.right_wrench = message

    def verifyCommand(self, cmd):
        cmd.TargetForce = max(RG2FT_MIN_FORCE, cmd.TargetForce)
        cmd.TargetForce = min(RG2FT_MAX_FORCE, cmd.TargetForce)
        
        cmd.TargetWidth = max(RG2FT_MIN_WIDTH, cmd.TargetWidth)
        cmd.TargetWidth = min(RG2FT_MAX_WIDTH, cmd.TargetWidth)

        return cmd

    def writeCommand(self, cmd: RG2FTCommand):
        self.cmd_pub.publish(cmd)

    def readState(self) -> RG2FTState:
        return self.state

    # def setProximityOffsets(self, left_offset, right_offset):
    #     self.driver.gripper.setProximityOffsets(left_offset, right_offset)

    # def zeroForceTorque(self, val):
    #     self.driver.gripper.zeroForceTorque(val)

    # def restartPowerCycle(self):
    #     self.driver.gripper.restartPowerCycle()

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """
        cmd = RG2FTCommand()
        cmd.TargetForce = int(force*20)
        cmd.TargetWidth = int((255 - position)*3)
        cmd.Control = 0x0001
        
        cmd = self.verifyCommand(cmd)
        self.cmd_pub.publish(cmd)
        
    def get_current_position(self) -> int:
        """Returns the current position as returned by the physical hardware."""
        # return self._get_var(self.POS)
        status = self.readState()
        scaled = 255 - status.ActualGripperWidth//3
        clipped = max(min(scaled, 255), 0)
        return clipped


def main():
    ip = "192.168.1.1"
    port = "502"
    # test open and closing the gripper
    gripper = OnRobotRG2FTROS()


if __name__ == "__main__":
    main()
