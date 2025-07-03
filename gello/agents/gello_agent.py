import os
import rospy
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[float, float]
    """The gripper config of GELLO. This is a tuple of (degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None, robot_type: Optional[str] = ""
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=self.joint_offsets,
            joint_signs=self.joint_signs,
            real=True,
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # xArm
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6, 7),
    #     joint_offsets=(
    #         2 * np.pi / 2,
    #         2 * np.pi / 2,
    #         2 * np.pi / 2,
    #         2 * np.pi / 2,
    #         -1 * np.pi / 2 + 2 * np.pi,
    #         1 * np.pi / 2,
    #         1 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, 1, 1, 1, 1, 1),
    #     gripper_config=(8, 279, 279 - 50),
    # ),
    # panda
    # "/dev/cu.usbserial-FT3M9NVB": DynamixelRobotConfig(
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6, 7),
    #     joint_offsets=(
    #         3 * np.pi / 2,
    #         2 * np.pi / 2,
    #         1 * np.pi / 2,
    #         4 * np.pi / 2,
    #         -2 * np.pi / 2 + 2 * np.pi,
    #         3 * np.pi / 2,
    #         4 * np.pi / 2,
    #     ),
    #     joint_signs=(1, -1, 1, 1, 1, -1, 1),
    #     gripper_config=(8, 195, 152),
    # ),
    # # Left UR
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6),
    #     joint_offsets=(
    #         0,
    #         1 * np.pi / 2 + np.pi,
    #         np.pi / 2 + 0 * np.pi,
    #         0 * np.pi + np.pi / 2,
    #         np.pi - 2 * np.pi / 2,port=port, start_joints=start_joints, robot_type=robot_type)
    #         -1 * np.pi / 2 + 2 * np.pi,
    #     ),
    #     joint_signs=(1, 1, -1, 1, 1, 1),
    #     gripper_config=(7, 20, -22),
    # ),
    # # Right UR
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6),
    #     joint_offsets=(
    #         np.pi + 0 * np.pi,
    #         2 * np.pi + np.pi / 2,
    #         2 * np.pi + np.pi / 2,
    #         2 * np.pi + np.pi / 2,
    #         1 * np.pi,
    #         3 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, -1, 1, 1, 1),
    #     gripper_config=(7, 286, 248),
    # ),
    # HRVIP UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISVJ0-if00-port0": DynamixelRobotConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[0*np.pi/2, 2*np.pi/2, 4*np.pi/2, 2*np.pi/2, 2*np.pi/2, 1*np.pi/2, 0.0 ],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        gripper_config=[18, -23],
    ),
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NNQT-if00-port0": DynamixelRobotConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[0*np.pi/2, 3*np.pi/2, 3*np.pi/2, 5*np.pi/2, 3*np.pi/2, 4*np.pi/2, 0.0 ],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        gripper_config=[18, -23],
    ),
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U2O1-if00-port0": DynamixelRobotConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[2*np.pi/2, 3*np.pi/2, 3*np.pi/2, 2*np.pi/2, 0*np.pi/2, 3*np.pi/2, 0.0 ],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        gripper_config=[148, 196],
    ),
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94VVSB-if00-port0": DynamixelRobotConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[4*np.pi/2, 2*np.pi/2, 2*np.pi/2, 3*np.pi/2, 1*np.pi/2, 1*np.pi/2, 0.0 ],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        gripper_config=[148, 196],
    ),
}

TYPE_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # HRVIP UR
    "ur": DynamixelRobotConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[0*np.pi/2, 2*np.pi/2, 4*np.pi/2, 2*np.pi/2, 2*np.pi/2, 1*np.pi/2, 0.0 ],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        gripper_config=[18, -23],
    ),
    # HRVIP Trossen
    "trossen": DynamixelRobotConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[2*np.pi/2, 3*np.pi/2, 3*np.pi/2, 2*np.pi/2, 0*np.pi/2, 2*np.pi/2, 0*np.pi/180 ],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        gripper_config=[18, -23],
    ),
}


class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
        robot_type: Optional[str] = "",
        publish_joint_state: Optional[bool] = True
    ):
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        elif port in PORT_CONFIG_MAP:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            assert robot_type in TYPE_CONFIG_MAP
            config = TYPE_CONFIG_MAP[robot_type]
            self._robot = config.make_robot(
                port=port, start_joints=start_joints
            )

        # Create a joint state which will be held when the GELLO is not engaged
        self.hold_state = None
        self.hold_state_saved = False

        # Create a publisher for the GELLO's joint state
        self.joint_pub = None
        if publish_joint_state:
            self.joint_pub = rospy.Publisher("gello/joint_state", JointState, queue_size=10)

    def act(self, obs: Dict[str, np.ndarray], moveto=False, hold=False, require_grip=True, goal=np.empty(7)) -> np.ndarray:
        joint_state = self._robot.get_joint_state() # Get GELLO joint state
        gripper_state = joint_state[-1] # Get gripper closedness as a proportion [0,1]
        joint_msg = JointState()
        joint_msg.position = joint_state
        if self.joint_pub:
            self.joint_pub.publish(joint_msg)
        try:
            if moveto: # (Be careful not to create a feedback loop between GELLO and follower robot)
                self._robot.set_torque_mode(True, self._robot._joint_ids[:-1]) # Turn on the GELLO controller motors, not including gripper
                self._robot.command_joint_state(goal) # Command GELLO to follower robot joint state
                self.hold_state_saved = False # Be ready to save a new position next time hold is toggled on
            elif hold or (require_grip and gripper_state < 0.10): # If commanded to hold, or gripper less than 10% closed
                if not self.hold_state_saved:
                    self.hold_state = joint_state # Save current position to be held
                    self.hold_state_saved = True # Don't repeat this until hold is toggled off and on again
                self._robot.set_torque_mode(True, self._robot._joint_ids[:-1]) # Turn on the GELLO controller motors, not including gripper
                self._robot.command_joint_state(self.hold_state) # Command GELLO to hold position
            else:
                self._robot.set_torque_mode(False, self._robot._joint_ids) # Turn off all GELLO controller motors
                self.hold_state_saved = False # Be ready to save a new position next time hold is toggled on
        except Exception as e:
            print(e)
            self._robot.set_torque_mode(False, self._robot._joint_ids) # Turn off all GELLO controller motors
        return joint_state