from typing import Dict

import time
import numpy as np

from gello.robots.robot import Robot

# ROS compatibility edits
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):

        print("ur_ros starting")

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

        # ROS compatibility edits
        rospy.init_node('gello_ros_control')

        joint_state_topic = "/joint_states"
        # joint_traj_topic = "/pos_joint_traj_controller/command"
        joint_pos_topic = "/joint_group_pos_controller/command"

        self._joint_state_subscriber = rospy.Subscriber(joint_state_topic, sensor_msgs.msg.JointState, self._joint_state_sub_callback)
        # self._joint_traj_publisher = rospy.Publisher(joint_traj_topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)
        self._joint_pos_publisher = rospy.Publisher(joint_pos_topic, std_msgs.msg.Float64MultiArray, queue_size=1)

        self._joint_state = None

    def _joint_state_sub_callback(self, message):
        """Save joint state received from _joint_state_subscriber"""
        self._joint_state = message

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:

        time.sleep(0.01)
        gripper_pos = self._joint_state[-1]
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = [x for x in self._joint_state.position] # copy to prevent list mutation
        ros_joint_indices = [3, 2, 0, 4, 5, 6, 1] # order in which joint_states message must be indexed to sort joints from base to gripper
        pos = [robot_joints[i] for i in ros_joint_indices] # rearrange joints to be in order from base to gripper
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        robot_joints = joint_state[:6]
        robot_joints_msg = std_msgs.msg.Float64MultiArray()
        robot_joints_msg.data = robot_joints
        self._joint_pos_publisher.publish(robot_joints_msg)

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


def main():
    ur = URRobot(no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
