import os
import rospy
from sensor_msgs.msg import JointState, Joy
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.agents.gello_agent import DynamixelRobotConfig, PORT_CONFIG_MAP, TYPE_CONFIG_MAP

class FakeGelloAgent(Agent):
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
                port=port, start_joints=start_joints, real=False
            )
        elif port in PORT_CONFIG_MAP:
            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(
                port=port, start_joints=start_joints, real=False
            )
        else:
            assert robot_type in TYPE_CONFIG_MAP
            config = TYPE_CONFIG_MAP[robot_type]
            self._robot = config.make_robot(
                port=port, start_joints=start_joints, real=False
            )

        # Create a joint state which will be held when the GELLO is not engaged
        self.hold_state = None
        self.hold_state_saved = False

        # Create a publisher for the GELLO's joint state
        self.joint_pub = None
        if publish_joint_state:
            self.joint_pub = rospy.Publisher("gello/joint_state", JointState, queue_size=10)
        
        # Create a subscriber for the fake GELLO inputs
        self.fake_gello_sub = rospy.Subscriber("gello/fake", Joy, self.fake_gello_callback)
        # Maintain a joint command array so velocities can be maintained
        self.joint_command = np.zeros_like(self._robot.get_joint_state())
        # Maintain a joint gain array so velocities can be scaled
        self.joint_gain = np.ones_like(self._robot.get_joint_state()) * 0.01

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
    
    def fake_gello_callback(self, msg):
        # Convert message to velocity commands, scaled [-1, 1] (Scaled [0, 1] for gripper)
        msg_joint_command = [
            (msg.buttons[6] - msg.buttons[7]), # Shoulder pan
            (msg.buttons[4] - msg.buttons[5]), # Shoulder tilt
            msg.axes[1], # Elbow tilt
            -msg.axes[0], # Forearm roll
            -msg.axes[4], # Wrist tilt
            -msg.axes[3], # Wrist roll
            (msg.buttons[2] - msg.buttons[1]), # Gripper open/close
        ]
        for i in range(len(self.joint_command)):
            self.joint_command[i] = self.joint_gain[i] * msg_joint_command[i]
        # Set arm joint commands by applying a diff to current values
        arm_state = np.add(self._robot._driver._joint_angles[:-1], self.joint_command[:-1])
        self._robot._driver._joint_angles[:-1] = arm_state
        # Set gripper commands by applying a scaled diff and constraining to accepted values
        min_gripper_state = min(self._robot.gripper_open_close[0], self._robot.gripper_open_close[1])
        max_gripper_state = max(self._robot.gripper_open_close[0], self._robot.gripper_open_close[1])
        gripper_state = self.joint_command[-1] * (self._robot.gripper_open_close[0] - self._robot.gripper_open_close[1]) + self._robot._driver._joint_angles[-1]
        gripper_state = min(max(gripper_state, min_gripper_state), max_gripper_state)
        self._robot._driver._joint_angles[-1] = gripper_state
        return