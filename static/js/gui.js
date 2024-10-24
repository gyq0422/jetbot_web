// JavaScript 文件: script.js
$(document).ready(function () {
    // 绑定按钮点击事件
    $('.control-button').on('click', function () {
        var command = $(this).attr('id');
        sendCommand(command);
    });

    // 发送控制命令到服务器的函数
    function sendCommand(command) {
        $.ajax({
            url: '/control',
            type: 'POST',
            data: { command: command },
            success: function (response) {
                console.log('Command sent: ' + command);
            },
            error: function (error) {
                console.error('Error sending command: ' + command);
            }
        });
    }
});
