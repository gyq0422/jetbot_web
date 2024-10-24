# Python 文件: app.py
from flask import Flask, request, jsonify, render_template
from pynput.keyboard import Controller, Key
import subprocess
import time

app = Flask(__name__)
keyboard = Controller()

# 启动roslaunch的全局变量
ros_process = None

# 启动roslaunch函数
def start_roslaunch():
    global ros_process
    if ros_process is None:
        ros_process = subprocess.Popen(['roslaunch', 'jetbot_ros', 'jetbotmini_keyboard.launch'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
        print("roslaunch 启动成功！")

# 激活终端窗口的函数（适用于 Linux）
def activate_terminal_window():
    try:
        # 使用 wmctrl 激活特定终端窗口
        subprocess.run(['wmctrl', '-a', 'jetbot_terminal'], check=True)
    except FileNotFoundError:
        print("wmctrl 工具未安装，无法激活终端窗口。")

# 主页路由
@app.route('/')
def index():
    return render_template('index.html')

# 控制命令的处理路由
@app.route('/send_command', methods=['POST'])
def control():
    global ros_process
    command = request.form.get('command')
    if command:
        try:
            # 如果roslaunch进程尚未启动，则启动它
            if ros_process is None:
                start_roslaunch()
                time.sleep(1)  # 等待进程启动

            # 激活终端窗口，确保按键输入到正确的地方
            activate_terminal_window()

            # 根据命令执行相应的按键操作
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
            else:
                return jsonify({'status': 'error', 'message': 'Unknown command'}), 400

            return jsonify({'status': 'success', 'command': command})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500
    else:
        return jsonify({'status': 'error', 'message': 'No command provided'}), 400

if __name__ == '__main__':
    print("服务器启动中...")
    app.run(host='0.0.0.0', port=5000, debug=True)
    print("服务器启动成功，可以在局域网中访问。")
