#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, request, jsonify
import rospy
from std_msgs.msg import Int32

app = Flask(__name__)

# 全局变量用于存储发布者
command_pub = None

@app.route('/health')
def health_check():
    """健康检查端点"""
    return jsonify({"status": "healthy"}), 200

@app.route('/ready')
def ready_check():
    """就绪检查端点"""
    return jsonify({"status": "ready"}), 200

@app.route('/command', methods=['POST'])
def handle_command():
    """处理控制命令"""
    try:
        data = request.get_json()
        command = data.get('command')
        
        if command is None:
            return jsonify({"error": "Missing command parameter"}), 400
            
        # 发布命令到 ROS 话题
        msg = Int32()
        msg.data = command
        command_pub.publish(msg)
        
        return jsonify({"status": "success", "command": command}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

def init_http_server(publisher):
    """初始化 HTTP 服务器"""
    global command_pub
    command_pub = publisher
    
    # 启动 Flask 应用
    app.run(host='0.0.0.0', port=8080, threaded=True) 