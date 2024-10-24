#!/usr/bin/env python

from __future__ import print_function

import os
import subprocess
import threading
import sys
from select import select


# 自动加载 ROS 环境
def load_ros_environment():
    ros_setup_path = "/opt/ros/noetic/setup.bash"

    if not os.path.exists(ros_setup_path):
        raise FileNotFoundError(f"Cannot find setup.bash at {ros_setup_path}. Is ROS installed correctly?")

    command = f"source {ros_setup_path} && env"
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, executable='/bin/bash')
    output, _ = proc.communicate()

    for line in output.decode('utf-8').splitlines():
        key, _, value = line.partition("=")
        os.environ[key] = value


# 加载 ROS 环境变量
load_ros_environment()

import rospy
from geometry_msgs.msg import Twist

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist_msg.linear.x = self.x * self.speed
            twist_msg.linear.y = self.y * self.speed
            twist_msg.linear.z = self.z * self.speed
            twist_msg.angular.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = self.th * self.turn

            self.condition.release()

            self.publisher.publish(twist_msg)

        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
        self.publisher.publish(twist_msg)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

# 封装的控制逻辑函数
def handle_command(command, pub_thread, speed, turn, speed_limit, turn_limit):
    x, y, z, th = 0, 0, 0, 0

    if command in moveBindings.keys():
        x, y, z, th = moveBindings[command]
    elif command in speedBindings.keys():
        speed = min(speed_limit, speed * speedBindings[command][0])
        turn = min(turn_limit, turn * speedBindings[command][1])
        print(vels(speed, turn))

    pub_thread.update(x, y, z, th, speed, turn)
    return speed, turn

if __name__ == "__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))

        while(1):
            key = getKey(settings, key_timeout)
            if key == '\x03':  # Ctrl-C
                break

            speed, turn = handle_command(key, pub_thread, speed, turn, speed_limit, turn_limit)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
