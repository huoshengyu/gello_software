# General Packages
from typing import Dict
import numpy as np
# ROS Packages
import rospy
import sensor_msgs.msg 
# Interbotix/Trossen Packages
from interbotix_xs_modules.arm import InterbotixManipulatorXS

from gello.robots.robot import Robot


class TrossenRobot(Robot):
    """A class representing a Trossen robot."""

    def __init__(self, robot_model: str = "vx300s", no_gripper: bool = False, gripper_type: str = "trossen"):

        [print("Launching Trossen %s..." % (robot_model))]
        self.robot = None
        while not self.robot:
            try:
                if not no_gripper:
                    self.robot = InterbotixManipulatorXS(robot_model, "arm", "gripper")
                else:
                    self.robot = InterbotixManipulatorXS(robot_model, "arm")
            except Exception as e:
                print(e)
                print(robot_ip)

        [print("connect") for _ in range(4)]

        self._free_drive = False
        # self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

        self.velocity = 0.5
        self.acceleration = 0.5
        self.dt = 1.0 / 500  # 2ms
        self.lookahead_time = 0.2
        self.gain = 100

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        # Copied with edits from interbotix_xs_modules
        # interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/gripper.py
        with self.robot.gripper.core.js_mutex:
            gripper_pos = self.robot.gripper.core.joint_states.position[self.robot.gripper.left_finger_index]
        return gripper_pos

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the follower robot.

        Returns:
            T: The current state of the follower robot.
        """
        # Copied with edits from interbotix_xs_modules
        # interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py
        robot_joints = [self.robot.arm.core.joint_states.position[self.robot.arm.core.js_index_map[name]] for name in self.robot.arm.group_info.joint_names]
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return np.array(pos)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the follower robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the follower robot to.
        """
        current_joint_state = self.get_joint_state()
        moving_time = max(2, (max(joint_state - current_joint_state)) / self.velocity)
        accel_time = min(moving_time / 2, max(0.3, moving_time / (self.acceleration * 4)))

        robot_joints = joint_state[:6]
        self.robot.arm.set_joint_positions(robot_joints, blocking=False)
        if self._use_gripper:
            gripper_pos = joint_state[-1]
            if gripper_pos < 0.6:
                self.robot.gripper.close(2.0)
            else:
                self.robot.gripper.open(2.0)
        rospy.sleep(self.dt)

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
    robot_model = "vx300s"
    gripper_type = "trossen"
    trossen = TrossenRobot(robot_model=robot_model, no_gripper=False, gripper_type=gripper_type)
    print(trossen)
    trossen.set_freedrive_mode(True)
    print(trossen.get_observations())


if __name__ == "__main__":
    main()
