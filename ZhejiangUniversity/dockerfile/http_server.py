from flask import Flask, jsonify, request
import paho.mqtt.client as mqtt
import json
import threading
import time

app = Flask(__name__)

# MQTT客户端配置
mqtt_client = mqtt.Client()
mqtt_client.connect("localhost", 1883, 60)

# 存储最新的数据
latest_data = {
    "PowerVoltage": None,
    "imu": None,
    "odom": None,
    "scan": None,
    "camera/rgb/image_raw/compressed": None
}

# MQTT回调函数
def on_message(client, userdata, msg):
    topic = msg.topic
    try:
        data = json.loads(msg.payload.decode())
        latest_data[topic] = data
    except:
        latest_data[topic] = msg.payload.decode()

mqtt_client.on_message = on_message

# 订阅所有需要的主题
for topic in latest_data.keys():
    mqtt_client.subscribe(topic)

# 启动MQTT客户端
mqtt_client.loop_start()

# HTTP路由
@app.route('/PowerVoltage', methods=['GET'])
def get_power_voltage():
    return jsonify(latest_data["PowerVoltage"])

@app.route('/imu', methods=['GET'])
def get_imu():
    return jsonify(latest_data["imu"])

@app.route('/odom', methods=['GET'])
def get_odom():
    return jsonify(latest_data["odom"])

@app.route('/scan', methods=['GET'])
def get_scan():
    return jsonify(latest_data["scan"])

@app.route('/camera/rgb/image_raw/compressed', methods=['GET'])
def get_camera():
    return jsonify(latest_data["camera/rgb/image_raw/compressed"])

@app.route('/cmd_vel', methods=['POST'])
def send_cmd_vel():
    data = request.json
    mqtt_client.publish("cmd_vel", json.dumps(data))
    return jsonify({"status": "success"})

@app.route('/turtle1/cmd_vel', methods=['POST'])
def send_turtle_cmd_vel():
    data = request.json
    mqtt_client.publish("turtle1/cmd_vel", json.dumps(data))
    return jsonify({"status": "success"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080) 