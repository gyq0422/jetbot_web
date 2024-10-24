# Python 文件: app.py
from flask import Flask, request, jsonify, render_template
from pynput.keyboard import Controller
import os
import threading
app = Flask(__name__)
keyboard = Controller()
def start_jetbot_keyboard():
    os.system("roslaunch jetbot_ros jetbotmini_keyboard.launch")

# 在一个新线程中启动键盘控制脚本，以免阻塞Flask服务器
threading.Thread(target=start_jetbot_keyboard, daemon=True).start()

# 主页路由
@app.route('/')
def index():
    return render_template('gui.html')

# 控制命令的处理路由
@app.route('/control', methods=['POST'])
def control():
    command = request.form.get('command')
    if command:
        try:
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
    app.run(host='0.0.0.0', port=5000, debug=True)
