#!/usr/bin/env python3
"""
=== ROBOT HIJACK / Manuelle Übernahme (Demo) ===

Stellt einen Angreifer dar, der sich in einen Roboter "gehackt" hat und ihn
manuell fernsteuert. Es gibt KEINEN kooperativen Schalter in der Autonomie –
der Roboter "weiß" nichts davon. Stattdessen flutet dieser Knoten die
Befehlstopics /<ns>/cmd_vel und /<ns>/steering mit hoher Frequenz (100 Hz)
und überstimmt damit die autonome Steuerung (autonomousbug.py tickt mit 20 Hz).

Effekt: Sobald gestartet, fährt der gewählte Roboter sofort auf 0 (bleibt
stehen) und gehorcht danach nur noch der Tastatur. Wird der Knoten beendet,
hört das Fluten auf und die Autonomie übernimmt von selbst wieder.

autonomousbug.py muss dafür NICHT geändert werden.

Nutzung:
  ros2 run my_robot_description manually.py              # -> Auswahl im Menü
  ros2 run my_robot_description manually.py -- robot_2   # -> direkt robot_2
  python3 manually.py 3                                  # -> direkt robot_3
"""
import sys, tty, termios, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

SPEED_STEP = 0.1
SPEED_MAX  = 4.0
STEER_STEP = 0.05
STEER_MAX  = 0.5

FLOOD_HZ   = 100.0   # Sende-/Flut-Frequenz; Autonomie laeuft mit 20 Hz

ROBOTS = ['robot_1', 'robot_2', 'robot_3']


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def choose_robot():
    """Roboter per Argument oder interaktivem Menue waehlen."""
    for arg in sys.argv[1:]:
        if arg.startswith('--'):          # ROS-Remapping-Args ueberspringen
            continue
        if arg in ROBOTS:
            return arg
        try:
            idx = int(arg)
            if 1 <= idx <= len(ROBOTS):
                return ROBOTS[idx - 1]
        except ValueError:
            pass

    print("\n╔══════════════════════════════════╗")
    print("║   Ziel-Roboter waehlen (HIJACK)  ║")
    print("╠══════════════════════════════════╣")
    for i, name in enumerate(ROBOTS, 1):
        print(f"║   {i}  =  {name:20s}     ║")
    print("╚══════════════════════════════════╝")
    while True:
        try:
            choice = input("\nNummer eingeben: ").strip()
            idx = int(choice)
            if 1 <= idx <= len(ROBOTS):
                return ROBOTS[idx - 1]
        except (ValueError, EOFError):
            pass
        print("Ungueltige Eingabe, nochmal.")


class Hijacker(Node):
    def __init__(self, robot_name: str):
        super().__init__('manual_driver')
        self.robot_name = robot_name

        self.cmd_pub   = self.create_publisher(
            Twist,   f'/{robot_name}/cmd_vel',  10)
        self.steer_pub = self.create_publisher(
            Float64, f'/{robot_name}/steering', 10)

        self.speed = 0.0
        self.steer = 0.0
        self.lock  = threading.Lock()
        self.running = True

        # Hochfrequenter Flut-Thread: ueberstimmt die Autonomie auf cmd_vel/steering
        self._flood = threading.Thread(target=self._flood_loop, daemon=True)
        self._flood.start()

    def _flood_loop(self):
        period = 1.0 / FLOOD_HZ
        while self.running and rclpy.ok():
            with self.lock:
                spd, st = self.speed, self.steer
            t = Twist()
            t.linear.x  = spd
            t.angular.z = -st
            self.cmd_pub.publish(t)
            s = Float64()
            s.data = st
            self.steer_pub.publish(s)
            time.sleep(period)

    def stop_flood(self):
        self.running = False
        if self._flood.is_alive():
            self._flood.join(timeout=0.5)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        # speed/steer stehen auf 0 -> der Flut-Thread haelt den Roboter sofort an
        print(f"\n  >>> ZUGRIFF auf {self.robot_name} <<<  Roboter angehalten, manuelle Kontrolle aktiv.\n")
        print(f"╔════════════════════════════════════╗")
        print(f"║  HIJACK: {self.robot_name:20s}      ║")
        print(f"╠════════════════════════════════════╣")
        print(f"║  W / S  =  vor / zurueck           ║")
        print(f"║  A / D  =  lenken L / R            ║")
        print(f"║  X      =  Stopp                   ║")
        print(f"║  Q      =  Beenden (Autonomie zur.)║")
        print(f"╚════════════════════════════════════╝\n")
        try:
            while True:
                key = get_key(settings).lower()

                if key == 'q':
                    with self.lock:
                        self.speed = 0.0
                        self.steer = 0.0
                    break

                with self.lock:
                    if key == 'w':
                        self.speed = min(self.speed + SPEED_STEP, SPEED_MAX)
                    elif key == 's':
                        self.speed = max(self.speed - SPEED_STEP, -SPEED_MAX)
                    elif key == 'x':
                        self.speed = 0.0

                    if key == 'a':
                        self.steer = min(self.steer + STEER_STEP, STEER_MAX)
                    elif key == 'd':
                        self.steer = max(self.steer - STEER_STEP, -STEER_MAX)
                    elif key not in ('w', 's'):
                        self.steer = 0.0

                    spd, st = self.speed, self.steer

                print(
                    f"\r  [{self.robot_name}|HIJACK]  speed: {spd:+.2f} m/s  "
                    f"|  steer: {st:+.2f} rad   ",
                    end='', flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    robot_name = choose_robot()
    print(f"\n→ Uebernehme {robot_name}")
    rclpy.init()
    node = Hijacker(robot_name)
    try:
        node.run()
    finally:
        node.stop_flood()                # Fluten stoppen -> Autonomie kommt zurueck
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
