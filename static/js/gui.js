document.addEventListener('DOMContentLoaded', (event) => {
    const socket = io.connect('http://localhost:5000');

    function sendCommand(command) {
        socket.emit('command', { 'command': command });
        console.log("Command sent: " + command);
    }

    // 按钮点击事件处理
    document.getElementById('forward').addEventListener('click', () => {
        sendCommand('i');  // 对应前进
    });

    document.getElementById('backward').addEventListener('click', () => {
        sendCommand(',');  // 对应后退
    });

    document.getElementById('left').addEventListener('click', () => {
        sendCommand('j');  // 对应左转
    });

    document.getElementById('right').addEventListener('click', () => {
        sendCommand('l');  // 对应右转
    });

    document.getElementById('stop').addEventListener('click', () => {
        sendCommand(' ');  // 停止
    });

    // 键盘输入事件处理
    window.addEventListener('keydown', (event) => {
        let key = event.key;
        let command = "";

        // 匹配原有键盘控制
        switch (key) {
            case "ArrowUp":
            case "w":
                command = 'i';  // 前进
                break;
            case "ArrowDown":
            case "s":
                command = ',';  // 后退
                break;
            case "ArrowLeft":
            case "a":
                command = 'j';  // 左转
                break;
            case "ArrowRight":
            case "d":
                command = 'l';  // 右转
                break;
            case "u":
                command = 'u';  // 前进 + 左转
                break;
            case "o":
                command = 'o';  // 前进 + 右转
                break;
            case "m":
                command = 'm';  // 后退 + 左转
                break;
            case ".":
                command = '.';  // 后退 + 右转
                break;
            case "U":
                command = 'U';  // 高级模式 前进 + 左转
                break;
            case "I":
                command = 'I';  // 高级模式 前进
                break;
            case "O":
                command = 'O';  // 高级模式 前进 + 右转
                break;
            case "J":
                command = 'J';  // 高级模式 左移
                break;
            case "L":
                command = 'L';  // 高级模式 右移
                break;
            case "M":
                command = 'M';  // 高级模式 后退 + 左移
                break;
            case "<":
                command = '<';  // 高级模式 后退
                break;
            case ">":
                command = '>';  // 高级模式 后退 + 右移
                break;
            case " ":
                command = ' ';  // 停止
                break;
            default:
                command = "";
        }

        if (command) {
            sendCommand(command);
        }
    });
});
