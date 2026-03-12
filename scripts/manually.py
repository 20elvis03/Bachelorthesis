#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
import sys, tty, termios

SPEED_STEP = 0.1
SPEED_MAX  = 4.0
STEER_STEP = 0.05
STEER_MAX  = 0.5

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Driver(Node):
    def __init__(self):
        super().__init__('manual_driver')
        self.cmd_pub   = self.create_publisher(Twist,   '/cmd_vel',  10)
        self.steer_pub = self.create_publisher(Float64, '/steering', 10)
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
        print("\n╔══════════════════════════════╗")
        print("║   my_robot Manualsteuerung   ║")
        print("╠══════════════════════════════╣")
        print("║  W / S  =  vor / zurück      ║")
        print("║  A / D  =  lenken L / R      ║")
        print("║  X      =  Stopp             ║")
        print("║  Q      =  Beenden           ║")
        print("╚══════════════════════════════╝\n")
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
                elif key != 'w' and key != 's':
                    self.steer = 0.0

                self.publish()
                print(f"\r  speed: {self.speed:+.2f} m/s  |  steer: {self.steer:+.2f} rad   ", end='', flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    driver = Driver()
    driver.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()