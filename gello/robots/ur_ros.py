from typing import Dict

import time
import numpy as np

from gello.robots.robot import Robot
# Note: Importing onrobot gripper messages is designed to 
# work when GELLO is used as a submodule of Morpheus
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output, Robotiq2FGripper_robot_input

# ROS compatibility edits
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg

class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.102", no_gripper: bool = False, gripper_type: str = "onrobot"):

        print("ur_ros starting")

        self._free_drive = False
        # self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper
        self._gripper_type = gripper_type

        # ROS compatibility edits
        rospy.init_node('gello_ros_control')

        joint_state_topic = "/joint_states"
        gripper_state_topic = "/onrobot_rg2ft_gripper/command"
        # joint_traj_topic = "/pos_joint_traj_controller/command"
        joint_pos_topic = "/joint_group_pos_controller/command"
        robotiq_gripper_topic = "/robotiq_2f_85_gripper/control"
        onrobot_gripper_topic = "/onrobot_rg2ft_gripper/command"

        self._joint_state_subscriber = rospy.Subscriber(joint_state_topic, sensor_msgs.msg.JointState, self._joint_state_sub_callback)
        # self._joint_traj_publisher = rospy.Publisher(joint_traj_topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)
        self._joint_pos_publisher = rospy.Publisher(joint_pos_topic, std_msgs.msg.Float64MultiArray, queue_size=1)
        self._robotiq_gripper_publisher = rospy.Publisher(robotiq_gripper_topic, Robotiq2FGripper_robot_output, queue_size=1)
        self._onrobot_gripper_publisher = rospy.Publisher(onrobot_gripper_topic, RG2FTCommand, queue_size=1)

        self._joint_state = None

    def _joint_state_sub_callback(self, message):
        """Save joint state received from _joint_state_subscriber"""
        if message is not None and len(message.position) == self.num_dofs():
            self._joint_state = message

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper and (self._gripper_type == "robotiq"):
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
        while self._joint_state is None:
            time.sleep(0.1)
        robot_joints = np.array([x for x in self._joint_state.position]) # copy to prevent list mutation
        if len(robot_joints) == 7:
            ros_joint_indices = [3, 2, 0, 4, 5, 6, 1] # order in which joint_states message must be indexed to sort joints from base to gripper
        elif len(robot_joints) == 6:
            ros_joint_indices = [2, 1, 0, 3, 4, 5]
        pos = np.array([robot_joints[i] for i in ros_joint_indices]) # rearrange joints to be in order from base to gripper
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        # Joint commands
        robot_joints = joint_state[:6]
        robot_joints_msg = std_msgs.msg.Float64MultiArray()
        robot_joints_msg.data = robot_joints
        self._joint_pos_publisher.publish(robot_joints_msg)

        # Get difference between current and target gripper state
        gripper_current_pos = self._get_gripper_pos()
        gripper_target_pos = joint_state[-1]
        gripper_delta_pos = gripper_current_pos - (gripper_target_pos/255) # Given as a proportion, [0,1]
        
        # Gripper commands
        if self._use_gripper:
            if self._gripper_type == "robotiq":
                command = Robotiq2FGripper_robot_output()
                command.rACT = 0x1
                command.rGTO = 0x1 # go to position
                command.rATR = 0x0 # No emergency release
                command.rSP = min(128, 128 * (gripper_delta_pos/0.1)) # speed
                command.rPR = gripper_target_pos # position (arbitrary, 0 - 255)
                command.rPR = min(command.rPR, 230)
                command.rPR = max(command.rPR, 0)
                command.rFR = min(20, 20 * (gripper_delta_pos/0.1)) # force (N)
                self._robotiq_gripper_publisher.publish(command)
            elif self._gripper_type == "onrobot":
                command = RG2FTCommand()
                command.TargetForce = int(min(200, 200 * (gripper_delta_pos/0.1))) # force (N/10, 0 - 400)
                command.TargetWidth = max(0, min(1000, (1 - gripper_target_pos) * 1000)) # position (mm/10, 0 - 1000)
                command.Control = 0x0001
                self._onrobot_gripper_publisher.publish(command)
            else:
                print("Invalid gripper type specified!")

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
