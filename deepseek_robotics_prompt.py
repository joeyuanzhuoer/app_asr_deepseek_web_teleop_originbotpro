#!/usr/bin/env python3
"""
DeepSeek Robotics Prompt Module
用于将自然语言语音指令转换为 Turtlesim 控制命令的 JSON 格式
"""

import json
import requests
from typing import Dict, Optional, Any
from datetime import datetime
import os
# os.getenv("DEEPSEEK_API_KEY", "

# DeepSeek API 配置
DEEPSEEK_API_URL = "https://api.deepseek.com/v1/chat/completions"
DEEPSEEK_API_KEY = 'sk-xxxxxxxx'
# 默认的系统提示词 - 详细的 Robotics Prompt
ROBOTICS_SYSTEM_PROMPT = """你是一个专业的机器人控制指令转换系统，专门用于将自然语言语音指令转换为 ROS2 Turtlesim 控制命令的 JSON 格式。

## 你的任务
将用户提供的自然语言指令（通常是语音识别的文本）转换为标准的 JSON 格式控制命令。

## 可用的命令类型

### 基础移动命令
1. **forward** - 前进（直线向前移动）
2. **backward** - 后退（直线向后移动）
3. **left** - 左转（原地逆时针旋转）
4. **right** - 右转（原地顺时针旋转）
5. **stop** - 停止（立即停止所有运动）

### 组合移动命令
6. **forward_left** - 前进左转（前进同时左转）
7. **forward_right** - 前进右转（前进同时右转）
8. **backward_left** - 后退左转（后退同时左转）
9. **backward_right** - 后退右转（后退同时右转）

### 高级命令
10. **circle** - 画圆（圆形轨迹移动）
11. **spin** - 旋转（原地旋转）
12. **move_with_duration** - 定时移动（指定持续时间）

### 速度控制
13. **set_speed** - 设置速度（调整整体速度倍数）

## JSON 输出格式

你必须严格按照以下格式输出 JSON，不要添加任何额外的文本、注释或说明：

```json
{
  "type": "turtlesim_command",
  "command": "命令类型",
  "data": {
    "linear_x": 0.0,
    "angular_z": 0.0,
    "duration": 0.0,
    "speed": 1.0
  },
  "timestamp": "ISO8601格式时间戳",
  "metadata": {
    "source": "voice",
    "original_text": "原始语音文本"
  }
}
```

## 参数说明

### linear_x (线性速度)
- **范围**: -2.0 到 2.0
- **单位**: 米/秒 (m/s)
- **正数**: 前进
- **负数**: 后退
- **0.0**: 不移动

### angular_z (角速度)
- **范围**: -2.0 到 2.0
- **单位**: 弧度/秒 (rad/s)
- **正数**: 左转（逆时针）
- **负数**: 右转（顺时针）
- **0.0**: 不旋转

### duration (持续时间)
- **范围**: >= 0.0
- **单位**: 秒
- **0.0**: 持续执行直到停止命令
- **> 0.0**: 执行指定时间后自动停止

### speed (速度倍数)
- **范围**: 0.1 到 2.0
- **默认**: 1.0（正常速度）
- **> 1.0**: 加速（如 1.5 表示 1.5 倍速度）
- **< 1.0**: 减速（如 0.5 表示 0.5 倍速度）

## 命令映射规则

### 中文指令映射
- "前进"、"向前"、"往前走" → forward (linear_x: 1.0, angular_z: 0.0)
- "后退"、"向后"、"往后走" → backward (linear_x: -1.0, angular_z: 0.0)
- "左转"、"向左转"、"向左" → left (linear_x: 0.0, angular_z: 1.0)
- "右转"、"向右转"、"向右" → right (linear_x: 0.0, angular_z: -1.0)
- "停止"、"停下"、"停" → stop (linear_x: 0.0, angular_z: 0.0)
- "前进左转"、"左前方" → forward_left (linear_x: 1.0, angular_z: 1.0)
- "前进右转"、"右前方" → forward_right (linear_x: 1.0, angular_z: -1.0)
- "画圆"、"转圈" → circle (linear_x: 1.0, angular_z: 1.0)
- "旋转"、"原地转" → spin (linear_x: 0.0, angular_z: 1.0)

### 速度修饰词
- "快速"、"快点"、"加速" → speed: 1.5
- "慢速"、"慢点"、"减速" → speed: 0.5
- "很慢"、"非常慢" → speed: 0.3
- "很快"、"非常快" → speed: 1.8

### 时间修饰词
- "X秒"、"持续X秒" → duration: X
- "一会儿"、"短暂" → duration: 1.0
- "长时间" → duration: 5.0

### 英文指令映射
- "forward", "go forward", "move forward" → forward
- "backward", "go back", "move back" → backward
- "left", "turn left" → left
- "right", "turn right" → right
- "stop", "halt" → stop
- "circle", "make circle" → circle
- "spin", "rotate" → spin

## 输出要求

1. **只输出 JSON**：不要添加任何前缀、后缀或说明文字
2. **严格格式**：确保 JSON 格式完全正确，可以被直接解析
3. **完整字段**：必须包含所有必需字段
4. **合理数值**：确保所有数值在有效范围内
5. **时间戳**：使用 ISO 8601 格式，例如 "2024-01-01T12:00:00"

## 示例

### 示例 1
**输入**: "前进"
**输出**:
```json
{
  "type": "turtlesim_command",
  "command": "forward",
  "data": {
    "linear_x": 1.0,
    "angular_z": 0.0,
    "duration": 0.0,
    "speed": 1.0
  },
  "timestamp": "2024-01-01T12:00:00",
  "metadata": {
    "source": "voice",
    "original_text": "前进"
  }
}
```

### 示例 2
**输入**: "快速前进2秒"
**输出**:
```json
{
  "type": "turtlesim_command",
  "command": "move_with_duration",
  "data": {
    "linear_x": 1.0,
    "angular_z": 0.0,
    "duration": 2.0,
    "speed": 1.5
  },
  "timestamp": "2024-01-01T12:00:00",
  "metadata": {
    "source": "voice",
    "original_text": "快速前进2秒"
  }
}
```

### 示例 3
**输入**: "左转"
**输出**:
```json
{
  "type": "turtlesim_command",
  "command": "left",
  "data": {
    "linear_x": 0.0,
    "angular_z": 1.0,
    "duration": 0.0,
    "speed": 1.0
  },
  "timestamp": "2024-01-01T12:00:00",
  "metadata": {
    "source": "voice",
    "original_text": "左转"
  }
}
```

## 重要提示

1. 如果用户输入无法识别或模糊，使用最接近的命令
2. 如果包含速度修饰词，调整 speed 字段
3. 如果包含时间信息，设置 duration 字段并使用 move_with_duration 命令
4. 始终保留原始文本在 metadata.original_text 中
5. 确保生成的 JSON 可以被 Python 的 json.loads() 直接解析

现在，请将用户提供的自然语言指令转换为 JSON 格式。只输出 JSON，不要添加任何其他内容。"""


class DeepSeekRoboticsPrompt:
    """DeepSeek Robotics Prompt 处理器"""
    
    def __init__(self, api_key: Optional[str] = None, api_url: str = DEEPSEEK_API_URL):
        """
        初始化 DeepSeek Robotics Prompt 处理器
        
        Args:
            api_key: DeepSeek API 密钥，如果不提供则从环境变量读取
            api_url: DeepSeek API 地址
        """
        self.api_key = api_key or DEEPSEEK_API_KEY
        self.api_url = api_url
        self.system_prompt = ROBOTICS_SYSTEM_PROMPT
        
        if not self.api_key:
            raise ValueError("DeepSeek API key is required. Set DEEPSEEK_API_KEY environment variable or pass api_key parameter.")
    
    def generate_command(self, voice_text: str, model: str = "deepseek-chat") -> Dict[str, Any]:
        """
        将自然语言语音指令转换为 Turtlesim 控制命令 JSON
        
        Args:
            voice_text: 语音识别的文本
            model: DeepSeek 模型名称，默认 "deepseek-chat"
        
        Returns:
            包含控制命令的字典，格式符合 TURTLESIM_JSON_FORMAT.md 规范
        
        Raises:
            ValueError: 如果 API 调用失败或返回无效 JSON
            requests.RequestException: 如果网络请求失败
        """
        if not voice_text or not voice_text.strip():
            raise ValueError("Voice text cannot be empty")
        
        # 准备 API 请求
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        payload = {
            "model": model,
            "messages": [
                {
                    "role": "system",
                    "content": self.system_prompt
                },
                {
                    "role": "user",
                    "content": voice_text.strip()
                }
            ],
            "temperature": 0.1,  # 低温度以获得更确定性的输出
            "max_tokens": 500,
            "response_format": {"type": "json_object"}  # 强制 JSON 输出
        }
        
        try:
            # 调用 DeepSeek API
            response = requests.post(
                self.api_url,
                headers=headers,
                json=payload,
                timeout=10
            )
            response.raise_for_status()
            
            result = response.json()
            
            # 提取生成的文本
            if "choices" not in result or not result["choices"]:
                raise ValueError("Invalid API response: no choices found")
            
            generated_text = result["choices"][0]["message"]["content"].strip()
            
            # 尝试解析 JSON
            # 如果返回的文本包含代码块标记，需要提取 JSON 部分
            if "```json" in generated_text:
                # 提取 JSON 代码块
                start = generated_text.find("```json") + 7
                end = generated_text.find("```", start)
                if end == -1:
                    end = len(generated_text)
                generated_text = generated_text[start:end].strip()
            elif "```" in generated_text:
                # 提取普通代码块
                start = generated_text.find("```") + 3
                end = generated_text.find("```", start)
                if end == -1:
                    end = len(generated_text)
                generated_text = generated_text[start:end].strip()
            
            # 解析 JSON
            command_json = json.loads(generated_text)
            
            # 验证和补充字段
            if "timestamp" not in command_json:
                command_json["timestamp"] = datetime.now().isoformat()
            
            if "metadata" not in command_json:
                command_json["metadata"] = {}
            
            if "original_text" not in command_json.get("metadata", {}):
                command_json.setdefault("metadata", {})["original_text"] = voice_text
            
            if "source" not in command_json.get("metadata", {}):
                command_json.setdefault("metadata", {})["source"] = "voice"
            
            # 验证必需字段
            required_fields = ["type", "command", "data"]
            for field in required_fields:
                if field not in command_json:
                    raise ValueError(f"Missing required field: {field}")
            
            # 验证 data 字段
            data = command_json["data"]
            required_data_fields = ["linear_x", "angular_z", "duration", "speed"]
            for field in required_data_fields:
                if field not in data:
                    raise ValueError(f"Missing required data field: {field}")
            
            # 验证数值范围
            linear_x = float(data["linear_x"])
            angular_z = float(data["angular_z"])
            duration = float(data["duration"])
            speed = float(data["speed"])
            
            if not (-2.0 <= linear_x <= 2.0):
                raise ValueError(f"linear_x out of range: {linear_x}")
            if not (-2.0 <= angular_z <= 2.0):
                raise ValueError(f"angular_z out of range: {angular_z}")
            if duration < 0.0:
                raise ValueError(f"duration must be >= 0: {duration}")
            if not (0.1 <= speed <= 2.0):
                raise ValueError(f"speed out of range: {speed}")
            
            return command_json
            
        except json.JSONDecodeError as e:
            raise ValueError(f"Failed to parse JSON from API response: {e}")
        except requests.RequestException as e:
            raise requests.RequestException(f"API request failed: {e}")
    
    def generate_command_safe(self, voice_text: str, model: str = "deepseek-chat") -> Dict[str, Any]:
        """
        安全版本的 generate_command，返回包含错误信息的字典而不是抛出异常
        
        Args:
            voice_text: 语音识别的文本
            model: DeepSeek 模型名称
        
        Returns:
            包含 success 字段的字典：
            - 成功: {"success": True, "command": {...}, ...}
            - 失败: {"success": False, "error": "...", "message": "..."}
        """
        try:
            command = self.generate_command(voice_text, model)
            return {
                "success": True,
                "command": command,
                "message": "命令生成成功"
            }
        except Exception as e:
            return {
                "success": False,
                "error": type(e).__name__,
                "message": str(e),
                "original_text": voice_text
            }


def create_default_prompt_handler() -> Optional[DeepSeekRoboticsPrompt]:
    """
    创建默认的 DeepSeek Robotics Prompt 处理器
    
    Returns:
        DeepSeekRoboticsPrompt 实例，如果 API key 未设置则返回 None
    """
    try:
        return DeepSeekRoboticsPrompt()
    except ValueError:
        return None


# 测试函数
if __name__ == "__main__":
    import sys
    
    # 检查 API key
    if not DEEPSEEK_API_KEY:
        print("错误: 未设置 DEEPSEEK_API_KEY 环境变量")
        print("请设置: export DEEPSEEK_API_KEY='your-api-key'")
        sys.exit(1)
    
    # 测试
    handler = DeepSeekRoboticsPrompt()
    
    test_cases = [
        "前进",
        "快速前进2秒",
        "快速后退2秒",
        "左转",
        "停止",
        "画圆"
    ]
    
    print("=" * 60)
    print("DeepSeek Robotics Prompt 测试")
    print("=" * 60)
    
    for test_text in test_cases:
        print(f"\n测试输入: {test_text}")
        print("-" * 60)
        try:
            result = handler.generate_command(test_text)
            print("生成结果:")
            print(json.dumps(result, ensure_ascii=False, indent=2))
        except Exception as e:
            print(f"错误: {e}")

