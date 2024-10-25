from typing import Dict

import numpy as np

from gello.robots.robot import Robot
from gello.robots.robotiq_gripper import RobotiqGripper
from gello.robots.onrobot_gripper_ros import OnRobotRG2FTROS
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
        import rtde_control
        import rtde_receive

        [print("in ur robot") for _ in range(4)]
        self.robot = None
        while not self.robot:
            try:
                self.robot = rtde_control.RTDEControlInterface(robot_ip)
            except Exception as e:
                print(e)
                print(robot_ip)

        self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not no_gripper:
            if gripper_type == "robotiq":

                self.gripper = RobotiqGripper()
                self.gripper.connect(device="/tmp/ttyUR")
                print("gripper connected")
            elif gripper_type == "onrobot":

                onrobot_ip = "192.168.1.1"
                onrobot_port = "502"
                self.gripper = OnRobotRG2FTROS()
                print("gripper connected")

            # gripper.activate()

        [print("connect") for _ in range(4)]

        self._free_drive = False
        # self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper
        self._gripper_type = gripper_type

        # ROS compatibility edits
        #rospy.init_node('gello_ros_control')

        # joint_traj_topic = "/pos_joint_traj_controller/command"
        joint_pos_topic = "/joint_group_pos_controller/command"

        # self._joint_traj_publisher = rospy.Publisher(joint_traj_topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)
        self._joint_pos_publisher = rospy.Publisher(joint_pos_topic, std_msgs.msg.Float64MultiArray, queue_size=1)

        self._joint_state = None

        # Joint command parameters
        self.velocity = 0.25
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
        import time

        time.sleep(0.01)
        gripper_pos = self.gripper.get_current_position()
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255 # Given as a proportion in range [0, 1]

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.r_inter.getActualQ()
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return np.array(pos)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        # Joint commands
        robot_joints = joint_state[:6]
        t_start = self.robot.initPeriod()
        self.robot.servoJ(
            robot_joints, self.velocity, self.acceleration, self.dt, self.lookahead_time, self.gain
        )
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
                command.rFR = 20 # force (N)
                gripper_command_pos = command.rPR
                gripper_command_speed = command.rSP
                gripper_command_force = command.rFR
                self.gripper.move(gripper_command_pos, gripper_command_speed, gripper_command_force)
            elif self._gripper_type == "onrobot":
                command = RG2FTCommand()
                command.TargetForce = int(min(200, 200 * (gripper_delta_pos/0.1))) # force (N/10, 0 - 400)
                command.TargetWidth = max(0, min(1000, (1 - gripper_target_pos) * 1000)) # position (mm/10, 0 - 1000)
                command.Control = 0x0001
                gripper_command_pos = command.TargetWidth
                gripper_command_speed = 0
                gripper_command_force = command.TargetForce
                self.gripper.move(gripper_command_pos, gripper_command_speed, gripper_command_force)
            else:
                print("Invalid gripper type specified!")
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
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

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
    robot_ip = "192.168.1.102"
    gripper_type = "onrobot"
    ur = URRobot(robot_ip, no_gripper=True, gripper_type=gripper_type)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
