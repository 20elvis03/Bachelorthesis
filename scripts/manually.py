#!/usr/bin/env python3
"""
Manuelle Steuerung für Multi-Robot Setup.

Nutzung:
  ros2 run my_robot_description manually.py              # → Auswahl im Menü
  ros2 run my_robot_description manually.py -- robot_1   # → direkt robot_1
  python3 manually.py robot_2                            # → direkt robot_2
"""
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
import sys, tty, termios

SPEED_STEP = 0.1
SPEED_MAX  = 4.0
STEER_STEP = 0.05
STEER_MAX  = 0.5

ROBOTS = ['robot_1', 'robot_2', 'robot_3']


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def choose_robot():
    """Interaktive Roboter-Auswahl oder per Kommandozeilen-Argument."""
    # Check command line args (skip ROS remapping args like --ros-args)
    for arg in sys.argv[1:]:
        if arg.startswith('--'):
            continue
        if arg in ROBOTS:
            return arg
        # Allow just the number: "1" → "robot_1"
        try:
            idx = int(arg)
            if 1 <= idx <= len(ROBOTS):
                return ROBOTS[idx - 1]
        except ValueError:
            pass

    # Interactive menu
    print("\n╔══════════════════════════════════╗")
    print("║     Roboter auswählen           ║")
    print("╠══════════════════════════════════╣")
    for i, name in enumerate(ROBOTS, 1):
        print(f"║  {i}  =  {name:20s}       ║")
    print("╚══════════════════════════════════╝")
    while True:
        try:
            choice = input("\nNummer eingeben: ").strip()
            idx = int(choice)
            if 1 <= idx <= len(ROBOTS):
                return ROBOTS[idx - 1]
        except (ValueError, EOFError):
            pass
        print("Ungültige Eingabe, nochmal.")


class Driver(Node):
    def __init__(self, robot_name: str):
        super().__init__('manual_driver')
        self.robot_name = robot_name

        # Topics unter dem Namespace des gewählten Roboters
        self.cmd_pub   = self.create_publisher(
            Twist,   f'/{robot_name}/cmd_vel',  10)
        self.steer_pub = self.create_publisher(
            Float64, f'/{robot_name}/steering', 10)

        self.speed = 0.0
        self.steer = 0.0

    def publish(self):
        t = Twist()
        t.linear.x  = self.speed
        t.angular.z = -self.steer
        self.cmd_pub.publish(t)

        s = Float64()
        s.data = self.steer
        self.steer_pub.publish(s)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(f"\n╔══════════════════════════════════╗")
        print(f"║  Steuerung: {self.robot_name:20s} ║")
        print(f"╠══════════════════════════════════╣")
        print(f"║  W / S  =  vor / zurück          ║")
        print(f"║  A / D  =  lenken L / R          ║")
        print(f"║  X      =  Stopp                 ║")
        print(f"║  Q      =  Beenden               ║")
        print(f"╚══════════════════════════════════╝\n")
        try:
            while True:
                key = get_key(settings).lower()

                if key == 'w':
                    self.speed = min(self.speed + SPEED_STEP, SPEED_MAX)
                elif key == 's':
                    self.speed = max(self.speed - SPEED_STEP, -SPEED_MAX)
                elif key == 'x':
                    self.speed = 0.0
                elif key == 'q':
                    self.speed = 0.0
                    self.steer = 0.0
                    self.publish()
                    break

                if key == 'a':
                    self.steer = min(self.steer + STEER_STEP, STEER_MAX)
                elif key == 'd':
                    self.steer = max(self.steer - STEER_STEP, -STEER_MAX)
                elif key not in ('w', 's'):
                    self.steer = 0.0

                self.publish()
                print(
                    f"\r  [{self.robot_name}]  speed: {self.speed:+.2f} m/s  "
                    f"|  steer: {self.steer:+.2f} rad   ",
                    end='', flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    robot_name = choose_robot()
    print(f"\n→ Steuere {robot_name}")

    rclpy.init()
    driver = Driver(robot_name)
    driver.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
