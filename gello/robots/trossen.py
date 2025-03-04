# General Packages
from typing import Dict
import numpy as np
# ROS Packages
import rospy
import sensor_msgs.msg 
# Interbotix/Trossen Packages
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.srv import JointGroupCommand

from gello.robots.robot import Robot


class TrossenRobot(Robot):
    """A class representing a Trossen robot."""

    def __init__(self, robot_ip: str = "192.168.1.102", no_gripper: bool = False, gripper_type: str = "trossen"):
        joint_state_topic = rospy.get_param("~joint_state_topic", "joint_states")
        self.joint_state_topic = rospy.subscriber(joint_state_topic, sensor_msgs.msg.JointState, 1)
        self.joint_state = None

        [print("in trossen robot") for _ in range(4)]
        self.robot = None
        while not self.robot:
            try:
                if not no_gripper:
                    self.robot = InterbotixManipulatorXS("vx300s", "arm", "gripper")
                else:
                    self.robot = InterbotixManipulatorXS("vx300s", "arm")
            except Exception as e:
                print(e)
                print(robot_ip)

        [print("connect") for _ in range(4)]

        self._free_drive = False
        # self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        return self.joint_state.position[-1]

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        pos = self.joint_state[:self.num_dofs()]
        return np.array(pos)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0 / 500  # 2ms
        lookahead_time = 0.2
        gain = 100

        robot_joints = joint_state[:6]
        t_start = self.robot.initPeriod()
        self.robot.arm.set_joint_positions(robot_joints)
        if self._use_gripper:
            gripper_pos = joint_state[-1]
            if gripper_pos < 0.6:
                self.robot.gripper.close(2.0)
            else:
                self.robot.gripper.open(2.0)
        self.robot.waitPeriod(t_start)

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            # self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            # self.robot.endFreedriveMode()

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }
    
    def joint_state_callback(self, msg):
        self.joint_state = msg


def main():
    robot_ip = "192.168.1.102"
    gripper_type = "onrobot"
    ur = URRobot(robot_ip, no_gripper=True, gripper_type=gripper_type)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
