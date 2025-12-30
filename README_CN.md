Project : app_asr_deepseek_web_teleop_originbotpro

https://github.com/joeyuanzhuoer/app_asr_deepseek_web_teleop_originbotpro.git

一个基于 Flask 的 Web 界面，用于控制 ROS2 Turtlesim，类似于 teleop_twist_keyboard，但拥有现代化的 Web 用户界面和易于使用的大按钮。

功能

🎮 基于 Web 的界面 - 可通过任何浏览器控制 Turtlesim

🔘 大按钮 - 易于在桌面、平板电脑或移动设备上使用

⌨️ 键盘支持 - 与 teleop_twist_keyboard 相同的按键布局

🎯 速度控制 - 动态调整线速度和角速度

📊 状态显示 - 实时显示连接和速度状态

📱 响应式设计 - 适用于所有屏幕尺寸

安装
前提条件

必须安装并加载 ROS2 Humble：

source /opt/ros/humble/setup.bash

Python 3.8+ 及以上版本，并已安装 pip

安装依赖项

cd web_teleop

pip install -r requirements.txt

或手动安装：

pip install flask rclpy

使用方法

步骤 1：启动 Turtlesim
终端 1：

source /opt/ros/humble/setup.bash

ros2 run turtlesim turtlesim_node

步骤 2：启动 Web 服务器
终端 2：

source /opt/ros/humble/setup.bash

cd web_teleop

python3 app.py

服务器将在 http://localhost:5000 启动

步骤 3：在浏览器中打开

打开您的 Web 浏览器并访问：

http://localhost:5000

或者，如果您从同一网络上的其他设备访问：

http://<您的 IP 地址>:5000

控制
移动按钮

按钮布局与 teleop_twist_keyboard 一致：

↖ (u) ↑ (i) ↗ (o)

← (j) ⏹ (k) → (l)

↙ (m) ↓ (,) ↘ (.)

点击按钮或使用键盘按键进行控制

按住按钮/按键可连续移动

松开按钮停止

速度控制
q/z：所有速度增加/减少 10%

w/x：线速度增加/减少 10%

角速度增加/减少 10%

停止
点击停止按钮（中心）

按 k 或空格键

键盘布局

与 teleop_twist_keyboard 相同：

移动：

u i o

j k l

m , .

速度控制：

q/z：最大速度增加/减少 10%

w/x：仅线速度增加/减少 10%

e/c：仅角速度增加/减少 10%

k 或空格键：立即停止

故障排除

未找到 ROS2

如果您看到“未找到 ROS2 Python 包”：

source /opt/ros/humble/setup.bash

python3 app.py

端口已被占用

如果端口 5000 被占用，请编辑 app.py 文件，并将：

app.run(host='0.0.0.0', port=5000, ...)

更改为其他端口（例如，5001）。

Turtlesim 无响应

检查 Turtlesim 是否正在运行：

ros2 node list

# 应该看到：/turtlesim

检查主题是否存在：

ros2 topic list

# 应该看到：/turtle1/cmd_vel

监控命令：

ros2 topic echo /turtle1/cmd_vel

# 在 Web 界面中移动海龟并验证消息是否显示

Web 界面未加载

检查防火墙设置

验证服务器是否正在运行（检查终端输出）

尝试访问 http://127.0.0.1:5000

文件结构

web_teleop/

├── app.py # Flask 服务器和 ROS2 节点

├── deepseek_robotics_prompt.py # Python request deepseek api llm service

├── requirements.txt # Python 依赖项

├── README_CN.md # 此文件

├── templates/

│ └── index.html # Web 界面 HTML

└── static/

├── style.css # 样式

└── script.js # 用于控件的 JavaScript

API 端点

GET / - 主 Web 界面

POST /cmd - 发送移动指令

{"command": "forward"}

GET /status - 获取当前状态和速度

{

"ros2_initialized": true,

"linear_speed": 1.0,

"angular_speed": 1.0

}

与 teleop_twist_keyboard 的比较

功能：Web Teleop teleop_twist_keyboard

界面：Web 浏览器/终端

按钮：大尺寸、可视键盘

移动设备支持：✅ 是 ❌ 否

远程访问：✅ 是 ❌ 否

按键布局：✅ 相同 ✅ 相同

速度控制：✅ 相同 ✅ 相同

开发
在调试模式下运行

编辑 app.py 并更改：

app.run(host='0.0.0.0', port=5000, debug=True, ...)

自定义
颜色：编辑 static/style.css

按钮布局：编辑 templates/index.html

速度默认值：编辑 DEFAULT_LINEAR_SPEED 和 app.py 中的 DEFAULT_ANGULAR_SPEED

主题名称：如果使用不同的海龟，请修改 app.py 中的 /turtle1/cmd_vel

许可
这是一个用于 ROS2 turtlesim 的实用工具。可免费用于教育和开发目的。
