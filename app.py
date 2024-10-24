from flask import Flask, render_template
from flask_socketio import SocketIO
from pynput.keyboard import Controller, Key
import threading
import os

# 创建Flask应用和SocketIO对象
app = Flask(__name__)
socketio = SocketIO(app)

# 创建键盘控制器对象
keyboard = Controller()

# 启动键盘控制ROS脚本
def start_jetbot_keyboard():
    os.system("roslaunch jetbot_ros jetbotmini_keyboard.launch")

# 在一个新线程中启动键盘控制脚本，以免阻塞Flask服务器
threading.Thread(target=start_jetbot_keyboard, daemon=True).start()

# 主页
@app.route('/')
def index():
    return render_template('gui.html')

# 处理前端的控制命令
@socketio.on('move_command')
def handle_move_command(command):
    if command == 'forward':
        keyboard.press('w')
        keyboard.release('w')
    elif command == 'backward':
        keyboard.press('s')
        keyboard.release('s')
    elif command == 'left':
        keyboard.press('a')
        keyboard.release('a')
    elif command == 'right':
        keyboard.press('d')
        keyboard.release('d')
    elif command == 'speed_left_increase':
        keyboard.press('u')
        keyboard.release('u')
    elif command == 'speed_left_decrease':
        keyboard.press('i')
        keyboard.release('i')
    elif command == 'speed_right_increase':
        keyboard.press('j')
        keyboard.release('j')
    elif command == 'speed_right_decrease':
        keyboard.press('k')
        keyboard.release('k')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
