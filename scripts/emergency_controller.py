#!/usr/bin/env python3
"""
Emergency Stop Controller – Multi-Robot
========================================
Tastatur-Steuerung für Emergency Stop aller Roboter.

Tasten:
  1 / 2 / 3   → Toggle Emergency Stop für robot_1 / _2 / _3
  A            → Toggle Emergency Stop für ALLE Roboter
  S            → Status anzeigen
  Q            → Beenden

Starten:
  ros2 run my_robot_gazebo emergency_controller

Oder direkt per Topic:
  ros2 topic pub /robot_1/emergency_stop std_msgs/msg/Bool "data: true" --once
  ros2 topic pub /emergency_stop_all     std_msgs/msg/Bool "data: true" --once
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


ROBOT_NAMES = ["robot_1", "robot_2", "robot_3"]


def get_key(settings):
    """Liest einzelne Taste ohne Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class EmergencyController(Node):
    """Interaktiver Emergency-Stop-Controller."""

    def __init__(self):
        super().__init__('emergency_controller')

        # Publisher für jeden Roboter
        self.robot_pubs = {}
        self.robot_stopped = {}
        for name in ROBOT_NAMES:
            topic = f'/{name}/emergency_stop'
            self.robot_pubs[name] = self.create_publisher(Bool, topic, 10)
            self.robot_stopped[name] = False
            self.get_logger().info(f'Publisher: {topic}')

        # Globaler Publisher
        self.all_pub = self.create_publisher(Bool, '/emergency_stop_all', 10)
        self.all_stopped = False

        self.get_logger().info('Emergency Controller bereit')

    def toggle_robot(self, name: str):
        """Toggle Emergency Stop für einen einzelnen Roboter."""
        if name not in self.robot_pubs:
            return
        self.robot_stopped[name] = not self.robot_stopped[name]
        msg = Bool()
        msg.data = self.robot_stopped[name]
        self.robot_pubs[name].publish(msg)
        status = '🛑 GESTOPPT' if self.robot_stopped[name] else '✅ FREIGEGEBEN'
        print(f'\r  {name}: {status}                    ')

    def toggle_all(self):
        """Toggle Emergency Stop für ALLE Roboter."""
        self.all_stopped = not self.all_stopped
        msg = Bool()
        msg.data = self.all_stopped

        # Global-Topic
        self.all_pub.publish(msg)

        # Auch einzelne Topics aktualisieren
        for name in ROBOT_NAMES:
            self.robot_stopped[name] = self.all_stopped
            self.robot_pubs[name].publish(msg)

        status = '🛑 ALLE GESTOPPT' if self.all_stopped else '✅ ALLE FREIGEGEBEN'
        print(f'\r  {status}                              ')

    def show_status(self):
        """Aktuellen Status anzeigen."""
        print('\r')
        print('  ┌─────────────────────────────────┐')
        print('  │       Emergency Status           │')
        print('  ├─────────────────────────────────┤')
        for name in ROBOT_NAMES:
            icon = '🛑' if self.robot_stopped[name] else '✅'
            state = 'STOP' if self.robot_stopped[name] else 'OK  '
            print(f'  │  {icon} {name:10s}  {state}           │')
        icon = '🛑' if self.all_stopped else '✅'
        state = 'STOP' if self.all_stopped else 'OK  '
        print(f'  │  {icon} {"GLOBAL":10s}  {state}           │')
        print('  └─────────────────────────────────┘')

    def run(self):
        """Hauptschleife – liest Tasten."""
        settings = termios.tcgetattr(sys.stdin)

        print('\n╔═══════════════════════════════════╗')
        print('║   Emergency Stop Controller       ║')
        print('╠═══════════════════════════════════╣')
        print('║  1/2/3  = Toggle Robot 1/2/3      ║')
        print('║  A      = Toggle ALLE             ║')
        print('║  S      = Status                  ║')
        print('║  Q      = Beenden                 ║')
        print('╚═══════════════════════════════════╝\n')

        self.show_status()

        try:
            while True:
                key = get_key(settings).lower()

                if key == '1':
                    self.toggle_robot('robot_1')
                elif key == '2':
                    self.toggle_robot('robot_2')
                elif key == '3':
                    self.toggle_robot('robot_3')
                elif key == 'a':
                    self.toggle_all()
                elif key == 's':
                    self.show_status()
                elif key == 'q':
                    # Alle freigeben vor dem Beenden
                    if self.all_stopped:
                        self.all_stopped = False
                        msg = Bool()
                        msg.data = False
                        self.all_pub.publish(msg)
                        for name in ROBOT_NAMES:
                            self.robot_stopped[name] = False
                            self.robot_pubs[name].publish(msg)
                    print('\r\n  Alle Emergency Stops freigegeben. Tschüss!')
                    break

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = EmergencyController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
