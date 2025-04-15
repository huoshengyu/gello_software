from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot: str = "sim_ur"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.102"
    no_gripper: bool = False


def launch_robot_server(args: Args):
    port = args.robot_port
    print("Launching nodes with robot type %s" % (args.robot))
    if args.robot == "sim_ur":
        from gello.robots.sim_robot import MujocoRobotServer
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_trossen":
        from gello.robots.sim_robot import MujocoRobotServer
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "trossen_vx300s" / "vx300s.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_panda":
        from gello.robots.sim_robot import MujocoRobotServer
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_xarm":
        from gello.robots.sim_robot import MujocoRobotServer
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobot
            robot = XArmRobot(ip=args.robot_ip)

        elif args.robot == "ur":
            from gello.robots.ur import URRobot
            robot = URRobot(robot_ip=args.robot_ip, no_gripper=args.no_gripper,
            		gripper_type="robotiq")
            
        elif args.robot == "ur_onrobot":
            from gello.robots.ur import URRobot
            robot = URRobot(robot_ip=args.robot_ip, no_gripper=args.no_gripper,
            		gripper_type="onrobot")
            
        elif args.robot == "ur_ros":
            from gello.robots.ur_ros import URRobot
            robot = URRobot(no_gripper=False, gripper_type="onrobot")

        elif args.robot == "trossen":
            from gello.robots.trossen import TrossenRobot
            robot = TrossenRobot(no_gripper=False, gripper_type="trossen")

        elif args.robot == "panda":
            from gello.robots.panda import PandaRobot
            robot = PandaRobot(robot_ip=args.robot_ip)

        elif args.robot == "bimanual_ur":
            from gello.robots.ur import URRobot
            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.10")
            _robot_r = URRobot(robot_ip="192.168.1.10")
            robot = BimanualRobot(_robot_l, _robot_r)

        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
