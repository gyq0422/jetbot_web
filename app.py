from flask import Flask, request, jsonify, render_template
from pynput.keyboard import Controller, Key
import time

app = Flask(__name__)
keyboard = Controller()

# 启动roslaunch的全局变量


# 主页路由
@app.route('/')
def index():
    return render_template('index.html')

# 控制命令的处理路由
@app.route('/control', methods=['POST'])
def control():
    command = request.form.get('command')
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

if __name__ == '__main__':
    print("服务器启动中...")
    app.run(host='0.0.0.0', port=5000, debug=True)
    print("服务器启动成功，可以在局域网中访问。")