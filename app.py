#!/usr/bin/env python3
"""
Flask Web Server for Turtlesim Teleop Control
Similar to teleop_twist_keyboard but with web interface
"""

import sys
import threading
import time
import os
from flask import Flask, render_template, request, jsonify

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:
    print("ERROR: ROS2 Python packages not found!")
    print("Please source ROS2 setup: source /opt/ros/humble/setup.bash")
    sys.exit(1)

# DeepSeek Robotics Prompt import
try:
    from deepseek_robotics_prompt import DeepSeekRoboticsPrompt, create_default_prompt_handler
    DEEPSEEK_AVAILABLE = True
except ImportError:
    print("WARNING: DeepSeek Robotics Prompt module not found. Voice command feature will be disabled.")
    DEEPSEEK_AVAILABLE = False

app = Flask(__name__)
# 配置 JSON 编码，不转义非 ASCII 字符（如中文）
app.config['JSON_AS_ASCII'] = False

# Global ROS2 node and publisher
ros_node = None
cmd_vel_publisher = None
ros_thread = None

# DeepSeek Robotics Prompt handler
deepseek_handler = None

# Default speeds (similar to teleop_twist_keyboard)
DEFAULT_LINEAR_SPEED = 1.0
DEFAULT_ANGULAR_SPEED = 1.0

# Current speeds
current_linear_speed = DEFAULT_LINEAR_SPEED
current_angular_speed = DEFAULT_ANGULAR_SPEED

# Speed increment (10% like teleop_twist_keyboard)
SPEED_INCREMENT = 0.1

# Active command thread for timed commands
active_command_thread = None
command_stop_event = threading.Event()


class TeleopNode(Node):
    """ROS2 Node for publishing cmd_vel messages"""
    
    def __init__(self):
        super().__init__('web_teleop_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Web teleop node started, publishing to /cmd_vel')
    
    def publish_twist(self, linear_x, angular_z):
        """Publish a Twist message"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular_z)
        self.publisher.publish(twist)


def init_ros2():
    """Initialize ROS2 in a separate thread"""
    global ros_node, cmd_vel_publisher
    
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = TeleopNode()
    cmd_vel_publisher = ros_node
    
    # Spin ROS2 node in a thread
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()
    
    print("ROS2 initialized successfully")


def init_deepseek():
    """Initialize DeepSeek Robotics Prompt handler"""
    global deepseek_handler
    
    if not DEEPSEEK_AVAILABLE:
        print("DeepSeek Robotics Prompt not available")
        return False
    
    try:
        deepseek_handler = create_default_prompt_handler()
        if deepseek_handler:
            print("DeepSeek Robotics Prompt initialized successfully")
            return True
        else:
            print("WARNING: DeepSeek API key not set. Voice command feature disabled.")
            print("Set DEEPSEEK_API_KEY environment variable to enable voice commands.")
            return False
    except Exception as e:
        print(f"WARNING: Failed to initialize DeepSeek: {e}")
        return False


def generate_command_description(command_data: dict, execution_result: dict) -> str:
    """
    生成可读的命令描述
    
    Args:
        command_data: 命令数据字典
        execution_result: 执行结果字典（包含已应用速度倍数的实际值）
    
    Returns:
        可读的命令描述字符串，例如："前进慢速2秒"
    """
    command_type = command_data.get('command', 'unknown')
    
    # 使用执行结果中的实际值（已应用速度倍数）
    linear_x = float(execution_result.get('linear_x', 0.0))
    angular_z = float(execution_result.get('angular_z', 0.0))
    duration = float(execution_result.get('duration', 0.0))
    speed = float(execution_result.get('speed', 1.0))
    
    # 命令类型映射
    command_names = {
        'forward': '前进',
        'backward': '后退',
        'left': '左转',
        'right': '右转',
        'stop': '停止',
        'forward_left': '前进左转',
        'forward_right': '前进右转',
        'backward_left': '后退左转',
        'backward_right': '后退右转',
        'circle': '画圆',
        'spin': '旋转',
        'move_with_duration': '定时移动',
        'set_speed': '设置速度'
    }
    
    # 构建描述
    parts = []
    
    # 速度描述（根据速度倍数）
    if speed > 1.2:
        parts.append('快速')
    elif speed < 0.5:
        parts.append('很慢')
    elif speed < 0.8:
        parts.append('慢速')
    
    # 方向描述
    if command_type in ['forward', 'backward', 'left', 'right', 'stop']:
        parts.append(command_names.get(command_type, command_type))
    elif command_type in ['forward_left', 'forward_right', 'backward_left', 'backward_right']:
        parts.append(command_names.get(command_type, command_type))
    elif command_type == 'circle':
        parts.append('画圆')
    elif command_type == 'spin':
        parts.append('旋转')
    elif command_type == 'move_with_duration':
        # 根据实际运动方向描述（使用已应用速度倍数的值）
        if abs(linear_x) > 0.1 or abs(angular_z) > 0.1:
            if linear_x > 0.1 and abs(angular_z) < 0.1:
                parts.append('前进')
            elif linear_x < -0.1 and abs(angular_z) < 0.1:
                parts.append('后退')
            elif abs(linear_x) < 0.1 and angular_z > 0.1:
                parts.append('左转')
            elif abs(linear_x) < 0.1 and angular_z < -0.1:
                parts.append('右转')
            elif linear_x > 0.1 and angular_z > 0.1:
                parts.append('前进左转')
            elif linear_x > 0.1 and angular_z < -0.1:
                parts.append('前进右转')
            elif linear_x < -0.1 and angular_z > 0.1:
                parts.append('后退左转')
            elif linear_x < -0.1 and angular_z < -0.1:
                parts.append('后退右转')
            else:
                parts.append('移动')
        else:
            parts.append('停止')
    else:
        parts.append(command_names.get(command_type, command_type))
    
    # 持续时间描述
    if duration > 0:
        parts.append(f'{duration}秒')
    
    # 组合描述
    description = ''.join(parts)
    
    return description


def execute_turtlesim_command(command_data: dict) -> dict:
    """
    执行 Turtlesim 控制命令
    
    Args:
        command_data: 命令数据字典，包含 linear_x, angular_z, duration, speed
    
    Returns:
        执行结果字典
    """
    global current_linear_speed, current_angular_speed, active_command_thread, command_stop_event
    
    if not cmd_vel_publisher:
        return {
            'success': False,
            'error': 'ROS2 not initialized'
        }
    
    # 提取命令数据
    data = command_data.get('data', {})
    linear_x = float(data.get('linear_x', 0.0))
    angular_z = float(data.get('angular_z', 0.0))
    duration = float(data.get('duration', 0.0))
    speed = float(data.get('speed', 1.0))
    
    # 应用速度倍数
    linear_x *= speed
    angular_z *= speed
    
    # 限制速度范围
    linear_x = max(-2.0, min(2.0, linear_x))
    angular_z = max(-2.0, min(2.0, angular_z))
    
    # 如果 duration > 0，在单独线程中执行定时命令
    if duration > 0.0:
        # 停止之前的命令
        command_stop_event.set()
        if active_command_thread and active_command_thread.is_alive():
            active_command_thread.join(timeout=0.5)
        
        # 创建新的事件
        command_stop_event = threading.Event()
        
        def timed_command():
            """定时执行命令"""
            start_time = time.time()
            while not command_stop_event.is_set():
                elapsed = time.time() - start_time
                if elapsed >= duration:
                    break
                
                cmd_vel_publisher.publish_twist(linear_x, angular_z)
                time.sleep(0.1)  # 10 Hz
            
            # 停止
            cmd_vel_publisher.publish_twist(0.0, 0.0)
        
        active_command_thread = threading.Thread(target=timed_command, daemon=True)
        active_command_thread.start()
        
        return {
            'success': True,
            'message': f'执行定时命令: {duration}秒',
            'linear_x': linear_x,
            'angular_z': angular_z,
            'duration': duration,
            'speed': speed
        }
    else:
        # 持续执行命令（直到收到停止命令）
        cmd_vel_publisher.publish_twist(linear_x, angular_z)
        
        return {
            'success': True,
            'message': '命令执行中（持续模式）',
            'linear_x': linear_x,
            'angular_z': angular_z,
            'duration': 0.0,
            'speed': speed
        }


@app.route('/')
def index():
    """Main page with control interface"""
    return render_template('index.html')


@app.route('/cmd', methods=['POST'])
def send_command():
    """Handle movement commands from web interface"""
    global current_linear_speed, current_angular_speed
    
    if not cmd_vel_publisher:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.json
    print(f"send cmd request: {data}")
    command = data.get('command', '')
    
    linear_x = 0.0
    angular_z = 0.0
    
    # Movement commands (similar to teleop_twist_keyboard layout)
    # Layout:
    #   u    i    o
    #   j    k    l
    #   m    ,    .
    
    if command == 'forward' or command == 'i':
        linear_x = current_linear_speed
        angular_z = 0.0
    elif command == 'backward' or command == ',':
        linear_x = -current_linear_speed
        angular_z = 0.0
    elif command == 'left' or command == 'j':
        linear_x = 0.0
        angular_z = current_angular_speed
    elif command == 'right' or command == 'l':
        linear_x = 0.0
        angular_z = -current_angular_speed
    elif command == 'forward_left' or command == 'u':
        linear_x = current_linear_speed
        angular_z = current_angular_speed
    elif command == 'forward_right' or command == 'o':
        linear_x = current_linear_speed
        angular_z = -current_angular_speed
    elif command == 'backward_left' or command == 'm':
        linear_x = -current_linear_speed
        angular_z = current_angular_speed
    elif command == 'backward_right' or command == '.':
        linear_x = -current_linear_speed
        angular_z = -current_angular_speed
    elif command == 'stop' or command == 'k':
        linear_x = 0.0
        angular_z = 0.0
    elif command == 'increase_speed' or command == 'q':
        # Increase max speeds by 10%
        current_linear_speed = min(current_linear_speed * (1 + SPEED_INCREMENT), 2.0)
        current_angular_speed = min(current_angular_speed * (1 + SPEED_INCREMENT), 2.0)
        return jsonify({
            'success': True,
            'linear_speed': current_linear_speed,
            'angular_speed': current_angular_speed,
            'message': f'Speeds increased to {current_linear_speed:.2f} m/s, {current_angular_speed:.2f} rad/s'
        })
    elif command == 'decrease_speed' or command == 'z':
        # Decrease max speeds by 10%
        current_linear_speed = max(current_linear_speed * (1 - SPEED_INCREMENT), 0.1)
        current_angular_speed = max(current_angular_speed * (1 - SPEED_INCREMENT), 0.1)
        return jsonify({
            'success': True,
            'linear_speed': current_linear_speed,
            'angular_speed': current_angular_speed,
            'message': f'Speeds decreased to {current_linear_speed:.2f} m/s, {current_angular_speed:.2f} rad/s'
        })
    elif command == 'increase_linear' or command == 'w':
        # Increase only linear speed by 10%
        current_linear_speed = min(current_linear_speed * (1 + SPEED_INCREMENT), 2.0)
        return jsonify({
            'success': True,
            'linear_speed': current_linear_speed,
            'angular_speed': current_angular_speed,
            'message': f'Linear speed increased to {current_linear_speed:.2f} m/s'
        })
    elif command == 'decrease_linear' or command == 'x':
        # Decrease only linear speed by 10%
        current_linear_speed = max(current_linear_speed * (1 - SPEED_INCREMENT), 0.1)
        return jsonify({
            'success': True,
            'linear_speed': current_linear_speed,
            'angular_speed': current_angular_speed,
            'message': f'Linear speed decreased to {current_linear_speed:.2f} m/s'
        })
    elif command == 'increase_angular' or command == 'e':
        # Increase only angular speed by 10%
        current_angular_speed = min(current_angular_speed * (1 + SPEED_INCREMENT), 2.0)
        return jsonify({
            'success': True,
            'linear_speed': current_linear_speed,
            'angular_speed': current_angular_speed,
            'message': f'Angular speed increased to {current_angular_speed:.2f} rad/s'
        })
    elif command == 'decrease_angular' or command == 'c':
        # Decrease only angular speed by 10%
        current_angular_speed = max(current_angular_speed * (1 - SPEED_INCREMENT), 0.1)
        return jsonify({
            'success': True,
            'linear_speed': current_linear_speed,
            'angular_speed': current_angular_speed,
            'message': f'Angular speed decreased to {current_angular_speed:.2f} rad/s'
        })
    else:
        return jsonify({'error': f'Unknown command: {command}'}), 400
    
    # Publish the command
    cmd_vel_publisher.publish_twist(linear_x, angular_z)
    
    return jsonify({
        'success': True,
        'command': command,
        'linear_x': linear_x,
        'angular_z': angular_z,
        'linear_speed': current_linear_speed,
        'angular_speed': current_angular_speed
    })


@app.route('/status', methods=['GET'])
def get_status():
    """Get current status and speeds"""
    return jsonify({
        'ros2_initialized': cmd_vel_publisher is not None,
        'linear_speed': current_linear_speed,
        'angular_speed': current_angular_speed,
        'deepseek_available': deepseek_handler is not None
    })


@app.route('/voice_command', methods=['POST'])
def handle_voice_command():
    """
    处理语音命令
    接收语音识别的文本，使用 DeepSeek 生成 JSON 命令，并执行
    
    Request JSON:
    {
        "text": "前进",
        "model": "deepseek-chat"  // 可选
    }
    
    Response JSON (成功):
    {
        "success": true,
        "message": "成功执行前进慢速2秒",
        "executed_at": "2024-01-01T12:00:00"
    }
    
    Response JSON (失败):
    {
        "success": false,
        "message": "执行失败: 错误描述",
        "executed_at": "2024-01-01T12:00:00"
    }
    """
    global deepseek_handler
    
    from datetime import datetime
    
    if not deepseek_handler:
        return jsonify({
            'success': False,
            'message': 'DeepSeek 未初始化',
            'executed_at': datetime.now().isoformat()
        }), 503
    
    if not cmd_vel_publisher:
        return jsonify({
            'success': False,
            'message': 'ROS2 未初始化',
            'executed_at': datetime.now().isoformat()
        }), 500
    
    # 获取请求数据
    data = request.json
    if not data:
        return jsonify({
            'success': False,
            'message': '请求格式错误: 必须是 JSON 格式',
            'executed_at': datetime.now().isoformat()
        }), 400
    
    voice_text = data.get('text', '').strip()
    if not voice_text:
        return jsonify({
            'success': False,
            'message': '请求格式错误: text 字段是必需的',
            'executed_at': datetime.now().isoformat()
        }), 400
    
    model = data.get('model', 'deepseek-chat')
    
    try:
        # 使用 DeepSeek 生成命令
        result = deepseek_handler.generate_command_safe(voice_text, model)
        
        if not result.get('success'):
            from datetime import datetime
            return jsonify({
                'success': False,
                'message': f"命令生成失败: {result.get('message', '未知错误')}",
                'executed_at': datetime.now().isoformat()
            }), 400
        
        # 获取生成的命令
        command_json = result['command']
        
        # 执行命令
        execution_result = execute_turtlesim_command(command_json)
        
        from datetime import datetime
        executed_at = datetime.now().isoformat()
        
        if not execution_result.get('success'):
            # 生成失败描述
            error_desc = f"执行失败: {execution_result.get('message', '未知错误')}"
            
            return jsonify({
                'success': False,
                'message': error_desc,
                'executed_at': executed_at
            }), 500
        
        # 生成成功描述（模板："成功执行命令慢速几秒"）
        command_description = generate_command_description(command_json, execution_result)
        success_desc = f"成功执行{command_description}"
        
        # 返回简化后的响应（仅三个字段）
        return jsonify({
            'success': True,
            'message': success_desc,
            'executed_at': executed_at
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': type(e).__name__,
            'message': str(e),
            'original_text': voice_text
        }), 500


@app.route('/voice_command/direct', methods=['POST'])
def handle_direct_command():
    """
    直接执行 JSON 格式的命令（不经过 DeepSeek）
    用于测试或直接发送命令
    
    Request JSON:
    {
        "type": "turtlesim_command",
        "command": "forward",
        "data": {
            "linear_x": 1.0,
            "angular_z": 0.0,
            "duration": 0.0,
            "speed": 1.0
        }
    }
    """
    if not cmd_vel_publisher:
        return jsonify({
            'success': False,
            'error': 'ROS2 not initialized'
        }), 500
    
    command_json = request.json
    if not command_json:
        return jsonify({
            'success': False,
            'error': 'Invalid request',
            'message': 'Request body must be JSON'
        }), 400
    
    # 验证命令格式
    if command_json.get('type') != 'turtlesim_command':
        return jsonify({
            'success': False,
            'error': 'Invalid command type',
            'message': 'type must be "turtlesim_command"'
        }), 400
    
    try:
        # 执行命令
        execution_result = execute_turtlesim_command(command_json)
        
        if not execution_result.get('success'):
            return jsonify({
                'success': False,
                'error': execution_result.get('error', 'Execution failed'),
                'message': execution_result.get('message', 'Failed to execute command')
            }), 500
        
        from datetime import datetime
        return jsonify({
            'success': True,
            'message': '命令执行成功',
            'command': command_json,
            'execution': execution_result,
            'executed_at': datetime.now().isoformat()
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': type(e).__name__,
            'message': str(e)
        }), 500


if __name__ == '__main__':
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Flask Web Teleop Server for Turtlesim')
    parser.add_argument(
        '--port',
        type=int,
        default=5000,
        help='Port to run the server on (default: 5000)'
    )
    parser.add_argument(
        '--host',
        type=str,
        default='0.0.0.0',
        help='Host to bind to (default: 0.0.0.0)'
    )
    args = parser.parse_args()
    
    print("=" * 60)
    print("Flask Web Teleop Server for Turtlesim")
    print("=" * 60)
    print("\nInitializing ROS2...")
    
    try:
        init_ros2()
        print("ROS2 initialized successfully!")
        
        # 初始化 DeepSeek（可选）
        print("\nInitializing DeepSeek Robotics Prompt...")
        init_deepseek()
        
        print(f"\nWeb server starting on http://{args.host}:{args.port}")
        print("Open your browser and navigate to the URL above")
        print("\nAPI Endpoints:")
        print("  POST /voice_command - Process voice command with DeepSeek")
        print("  POST /voice_command/direct - Execute direct JSON command")
        print("  POST /cmd - Send movement command")
        print("  GET /status - Get server status")
        print("\nPress Ctrl+C to stop the server")
        print("=" * 60)
        
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
        # 停止所有活动命令
        if active_command_thread and active_command_thread.is_alive():
            command_stop_event.set()
            active_command_thread.join(timeout=1.0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if rclpy.ok():
            if ros_node:
                ros_node.destroy_node()
            rclpy.shutdown()

