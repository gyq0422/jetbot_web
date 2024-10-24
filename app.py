# -*- coding: utf-8 -*-
from importlib import import_module
import os
from flask import Response


# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from camera import Camera

# Raspberry Pi camera module (requires picamera package)
# from camera_pi import Camera
from teleop_twist_keyboard import handle_command, PublishThread
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import rospy
from geometry_msgs.msg import Twist

# 初始化 Flask 应用和 SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

# 初始化 ROS 节点和 Publisher
rospy.init_node('web_teleop_twist_keyboard', anonymous=True)

speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 1.0)
speed_limit = rospy.get_param("~speed_limit", 1000)
turn_limit = rospy.get_param("~turn_limit", 1000)
repeat = rospy.get_param("~repeat_rate", 0.0)

pub_thread = PublishThread(repeat)

@app.route('/')
def gui():
    return render_template('gui.html')


@socketio.on('command')
def handle_command_from_web(data):
    global speed, turn
    command = data['command']
    speed, turn = handle_command(command, pub_thread, speed, turn, speed_limit, turn_limit)

   

def gen(camera):

    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():

    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
