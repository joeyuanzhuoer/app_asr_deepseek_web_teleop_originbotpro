#!/usr/bin/env python3
"""
测试语音命令 API 的脚本
显示格式化的 JSON 输出，中文正常显示
"""

import sys
import json
import requests
from datetime import datetime

def test_voice_command(text, port=8888, host='localhost'):
    """测试语音命令"""
    url = f"http://{host}:{port}/voice_command"
    
    print("=" * 60)
    print(f"测试语音命令: {text}")
    print(f"API 地址: {url}")
    print("=" * 60)
    
    try:
        response = requests.post(
            url,
            json={"text": text, "model": "deepseek-chat"},
            headers={"Content-Type": "application/json"},
            timeout=10
        )
        
        print(f"\n状态码: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            print("\n✅ 成功响应:")
            print(json.dumps(result, ensure_ascii=False, indent=2))
            
            # 显示关键信息（简化格式）
            if result.get('success'):
                print("\n📋 命令摘要:")
                print(f"  执行状态: {result.get('message', 'N/A')}")
                print(f"  执行时间: {result.get('executed_at', 'N/A')}")
            else:
                print("\n❌ 执行失败:")
                print(f"  错误信息: {result.get('message', 'N/A')}")
                print(f"  执行时间: {result.get('executed_at', 'N/A')}")
        else:
            print(f"\n❌ 错误响应:")
            try:
                error = response.json()
                print(json.dumps(error, ensure_ascii=False, indent=2))
            except:
                print(response.text)
                
    except requests.exceptions.ConnectionError:
        print(f"\n❌ 连接失败: 无法连接到 {url}")
        print("   请确保服务器正在运行")
    except requests.exceptions.Timeout:
        print(f"\n❌ 请求超时")
    except Exception as e:
        print(f"\n❌ 错误: {e}")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='测试语音命令 API')
    parser.add_argument(
        'text',
        nargs='?',
        default='前进',
        help='语音识别的文本（默认: 前进）'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8888,
        help='服务器端口（默认: 8888）'
    )
    parser.add_argument(
        '--host',
        type=str,
        default='localhost',
        help='服务器地址（默认: localhost）'
    )
    
    args = parser.parse_args()
    
    test_voice_command(args.text, args.port, args.host)


if __name__ == '__main__':
    main()

