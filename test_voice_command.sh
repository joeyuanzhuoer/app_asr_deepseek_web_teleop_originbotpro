#!/bin/bash
# 测试语音命令的脚本，使用 jq 美化输出

# 检查 jq 是否安装
if ! command -v jq &> /dev/null; then
    echo "jq 未安装，使用 Python 格式化输出..."
    python3 -c "
import sys
import json
import requests

url = sys.argv[1] if len(sys.argv) > 1 else 'http://localhost:8888/voice_command'
text = sys.argv[2] if len(sys.argv) > 2 else '前进'

response = requests.post(url, json={'text': text}, headers={'Content-Type': 'application/json'})
result = response.json()
print(json.dumps(result, ensure_ascii=False, indent=2))
" "$@"
else
    # 使用 jq 格式化输出
    PORT=${1:-8888}
    TEXT=${2:-"前进"}
    
    curl -s -X POST "http://localhost:${PORT}/voice_command" \
        -H "Content-Type: application/json" \
        -d "{\"text\": \"${TEXT}\"}" | jq .
fi

