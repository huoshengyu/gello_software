import datetime
import glob
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

from gello.agents.agent import BimanualAgent, DummyAgent
from gello.agents.gello_agent import GelloAgent, PORT_CONFIG_MAP, TYPE_CONFIG_MAP
from gello.data_utils.format_obs import save_frame
from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.zmq_core.robot_node import ZMQClientRobot


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: str = ""  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False


def main(args):
    # Start robot client, connect to robot port
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    # Handle bimanual agent/controller case
    if args.bimanual:
        if args.agent == "gello":
            # dynamixel control box port map (to distinguish left and right gello)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0"
            left_agent = GelloAgent(port=left)
            right_agent = GelloAgent(port=right)
            agent = BimanualAgent(left_agent, right_agent)
        elif args.agent == "quest":
            from gello.agents.quest_agent import SingleArmQuestAgent

            left_agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
            right_agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="r")
            agent = BimanualAgent(left_agent, right_agent)
            # raise NotImplementedError
        elif args.agent == "spacemouse":
            from gello.agents.spacemouse_agent import SpacemouseAgent

            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            left_agent = SpacemouseAgent(
                robot_type=args.robot_type,
                device_path=left_path,
                verbose=args.verbose,
                invert_button=False,
            )
            right_agent = SpacemouseAgent(
                robot_type=args.robot_type,
                device_path=right_path,
                verbose=args.verbose,
                invert_button=True,
            )
            agent = BimanualAgent(left_agent, right_agent)
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([0, -90, -90, -90, 90, 0, 0])
        reset_joints_right = np.deg2rad([0, -90, 90, -90, -90, 0, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)

    # Handle single agent/controller case
    else:
        if args.agent == "gello":
            gello_port = args.gello_port
            if gello_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    gello_port = usb_ports[0]
                    for port in usb_ports:
                        if port in PORT_CONFIG_MAP:
                            gello_port = port
                    print(f"using port {gello_port}")
                else:
                    raise ValueError(
                        "No gello port found, please specify one or plug in gello"
                    )
            if args.start_joints is None:
                print("Using default starting joint states for robot type: " + args.robot_type)
                # UR5e arched home position
                if args.robot_type == "ur":
                    reset_joints = np.deg2rad(
                        [180, -90, 90, -90, -90, 0, 0]
                    )
                # Trossen sleep position, adjusted for GELLO sleep pose
                elif args.robot_type == "trossen":
                    reset_joints = np.deg2rad(
                        [0, -97.559, 82.617, 0, 57.129, 0, 0]
                    )
                else:
                    reset_joints = np.deg2rad(
                        [180, -90, 90, -90, -90, 0, 0]
                    )
                print(reset_joints)
            else:
                reset_joints = args.start_joints
            agent = GelloAgent(port=gello_port, start_joints=args.start_joints, robot_type=args.robot_type)
        elif args.agent == "quest":
            from gello.agents.quest_agent import SingleArmQuestAgent

            agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
        elif args.agent == "spacemouse":
            from gello.agents.spacemouse_agent import SpacemouseAgent

            agent = SpacemouseAgent(robot_type=args.robot_type, verbose=args.verbose)
        elif args.agent == "dummy" or args.agent == "none":
            agent = DummyAgent(num_dofs=robot_client.num_dofs())
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")
        
    # Start agent
    try:
        # Move robot to start position
        print("Moving robot to start position")
        robot_pos = env.get_obs()["joint_positions"]
        if reset_joints.shape == robot_pos.shape:
            max_delta = (np.abs(robot_pos - reset_joints)).max()
            steps = max(int(max_delta * 100), 100)

            for jnt in np.linspace(robot_pos, reset_joints, steps):
                env.step(jnt)
                time.sleep(0.001)
            time.sleep(0.5)

        # Prepare to move GELLO to start position
        print("Moving GELLO to start position")
        start_pos = reset_joints
        gello_pos = agent.act(env.get_obs())
        start_pos = start_pos[0:len(gello_pos)]

        # Ensure start position and joint position have same dimensions
        print(f"Start pos: {len(start_pos)}", f"Joints: {len(gello_pos)}")
        assert len(start_pos) == len(
            gello_pos
        ), f"agent output dim = {len(start_pos)}, but env dim = {len(gello_pos)}"

        # Limit distance of controller from start position
        abs_deltas = np.abs(start_pos - gello_pos)
        delta_limit = 0.8
        if (abs_deltas > delta_limit).any():
            print()
            print("Controller joints are too far from start position:")

            # Print which joints are too far
            id_mask = abs_deltas > delta_limit
            print()
            ids = np.arange(len(id_mask))[id_mask]
            for i, delta, start_j, current_j in zip(
                ids,
                abs_deltas[id_mask],
                start_pos[id_mask],
                gello_pos[id_mask],
            ):
                print(
                    f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{current_j:4.3f} , startpos: \t{start_j:4.3f}"
                )
            agent._robot.set_torque_mode(False, agent._robot._joint_ids)
            return

        # Set GELLO start params
        start_move_time = 4.0 # seconds
        start_move_steps = int(start_move_time * env._rate.rate) # Get steps based on time allotted
        print(start_move_steps)
        delta_limit_per_second = np.pi/8 # Desired max radians/sec
        obs = env.get_obs()
        start_joints = agent.act(obs) # Initialize array of GELLO joints
        command_joints = start_pos # Use start pos as target position for GELLO
        command_joints = command_joints[0:len(start_joints)]
        joint_delta = command_joints - start_joints
        max_joint_delta = max(np.abs(joint_delta).max(), 0.000001)
        delta_proportion_per_second = min(1.0, abs(delta_limit_per_second / max_joint_delta)) # Scale command to obey delta limit per step
        
        # Move GELLO to start position
        print("Start GELLO move")
        start_time = datetime.datetime.now()
        while (datetime.datetime.now() - start_time).total_seconds() < start_move_time/2:
            obs = env.get_obs()
            seconds_elapsed = (datetime.datetime.now() - start_time).total_seconds()
            delta_proportion_total = min(1.0, abs(delta_proportion_per_second * seconds_elapsed)) # Find target progress toward final target position
            delta = joint_delta * delta_proportion_total # Find current target position
            action = agent.act(obs, moveto=True, goal=(start_joints + delta)) # Command GELLO to move with scaled command, get GELLO position for next loop
            env._rate.sleep()

        end_time = datetime.datetime.now()
        elapsed_time = end_time - start_time
        print(elapsed_time)

        # Move follower robot to GELLO while GELLO holds position
        print("Start Robot move")
        start_time = datetime.datetime.now()
        previous_step_time = datetime.datetime.now()
        obs = env.get_obs()
        current_joints = obs["joint_positions"]
        while (datetime.datetime.now() - start_time).total_seconds() < start_move_time/2:
            current_step_time = datetime.datetime.now()
            seconds_elapsed_this_step = (current_step_time - previous_step_time).total_seconds()
            previous_step_time = current_step_time
            delta_limit_this_step = delta_limit_per_second * seconds_elapsed_this_step
            obs = env.get_obs()
            command_joints = agent.act(obs, hold=True) # Command GELLO to hold position, use GELLO position as target
            command_joints = command_joints[0:len(current_joints)]
            delta = command_joints - current_joints
            max_joint_delta = np.abs(delta).max()
            if max_joint_delta > delta_limit_this_step:
                delta = (delta / max_joint_delta) * delta_limit_this_step # Scale command to obey delta limit per step
            env.step(current_joints + delta) # Command follower robot to move with scaled command
            current_joints = current_joints + delta # Update current joints to continue stepping closer to target

        end_time = datetime.datetime.now()
        elapsed_time = end_time - start_time
        print(elapsed_time)

        # Prepare to start user control
        obs = env.get_obs()
        joints = obs["joint_positions"]
        action = agent.act(obs)
        action = action[0:len(joints)]

        # Limit distance of controller from robot (Post-move)
        abs_deltas = np.abs(action - joints)
        delta_limit = 0.5
        if (abs_deltas > delta_limit).any():
            print("Start position error between controller and robot is too big:")

            # Print which joint deltas are too big
            joint_index = np.where(abs_deltas > delta_limit)
            for j in joint_index:
                print(
                    f"Joint [{j}]: \t delta: {action[j] - joints[j]} , leader: \t{action[j]} , follower: \t{joints[j]}"
                )
            agent._robot.set_torque_mode(False, agent._robot._joint_ids)
            return

        if args.use_save_interface:
            from gello.data_utils.keyboard_interface import KBReset

            kb_interface = KBReset()

        print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))

        save_path = None
        start_time = time.time()
        while True:
            num = time.time() - start_time
            message = f"\rTime passed: {round(num, 2)}          "
            print_color(
                message,
                color="white",
                attrs=("bold",),
                end="",
                flush=True,
            )
            action = agent.act(obs, require_grip=False) # Set to True to hold position. Currently doesn't work well due to weak Dynamixel motors.
            action = action[0:len(joints)]
            dt = datetime.datetime.now()
            if args.use_save_interface:
                state = kb_interface.update()
                if state == "start":
                    dt_time = datetime.datetime.now()
                    save_path = (
                        Path(args.data_dir).expanduser()
                        / args.agent
                        / dt_time.strftime("%m%d_%H%M%S")
                    )
                    save_path.mkdir(parents=True, exist_ok=True)
                    print(f"Saving to {save_path}")
                elif state == "save":
                    assert save_path is not None, "something went wrong"
                    save_frame(save_path, dt, obs, action)
                elif state == "normal":
                    save_path = None
                else:
                    raise ValueError(f"Invalid state {state}")
            obs = env.step(action)
    except KeyboardInterrupt:
        # Turn off all GELLO controller motors
        print("Shutting off GELLO motors")
        agent._robot.set_torque_mode(False, agent._robot._joint_ids)


if __name__ == "__main__":
    main(tyro.cli(Args))
