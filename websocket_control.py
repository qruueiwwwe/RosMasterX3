#!/usr/bin/env python3
import asyncio
import websockets
import json
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading

class WebSocketControl:
    def __init__(self, host='0.0.0.0', port=8765):
        self.host = host
        self.port = port
        self.clients = set()
        
        # ROS发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.keyboard_event_pub = rospy.Publisher('/keyboard_event', String, queue_size=10)
        
        # 启动WebSocket服务器
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    async def handle_client(self, websocket, path):
        """处理WebSocket客户端连接"""
        self.clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_command(data)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        'status': 'error',
                        'message': '无效的JSON格式'
                    }))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)

    async def process_command(self, data):
        """处理接收到的命令"""
        command_type = data.get('type')
        
        if command_type == 'cmd_vel':
            # 处理速度控制命令
            twist_msg = Twist()
            twist_msg.linear.x = float(data.get('linear_x', 0.0))
            twist_msg.linear.y = float(data.get('linear_y', 0.0))
            twist_msg.linear.z = float(data.get('linear_z', 0.0))
            twist_msg.angular.x = float(data.get('angular_x', 0.0))
            twist_msg.angular.y = float(data.get('angular_y', 0.0))
            twist_msg.angular.z = float(data.get('angular_z', 0.0))
            
            self.cmd_vel_pub.publish(twist_msg)
            
        elif command_type == 'keyboard':
            # 处理键盘事件
            key = data.get('key')
            if key:
                self.keyboard_event_pub.publish(String(key))
                
        elif command_type == 'emergency_stop':
            # 处理紧急停止命令
            twist_msg = Twist()  # 所有速度设为0
            self.cmd_vel_pub.publish(twist_msg)
            self.keyboard_event_pub.publish(String('space'))

    def _run_server(self):
        """运行WebSocket服务器"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        start_server = websockets.serve(
            self.handle_client,
            self.host,
            self.port
        )
        loop.run_until_complete(start_server)
        loop.run_forever()

    def stop(self):
        """停止WebSocket服务器"""
        for client in self.clients:
            asyncio.get_event_loop().create_task(client.close())
        self.clients.clear()

if __name__ == "__main__":
    try:
        rospy.init_node('websocket_control', anonymous=True)
        ws_control = WebSocketControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 