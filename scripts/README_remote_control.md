# 移动网页远程控制（实验性）

## 架构

```
手机浏览器  <--HTTP-->  Flask桥接脚本  <--UDP-->  rl_sim_mujoco
```

- **C++ 端** (`rl_sim_mujoco`)：内嵌 UDP 服务器，监听 `0.0.0.0:9876`
- **Python 端** (`udp_remote_bridge.py`)：Flask HTTP 服务器，接收手机浏览器的 POST 请求，转发为 UDP 报文
- **移动端网页**：内嵌在 Flask 中，支持虚拟摇杆 + 状态切换按钮

## 协议

UDP 报文格式（纯文本键值对）：
```
x=0.50&y=0.00&yaw=0.20&state=locomotion
```

- `x` / `y` / `yaw`：速度指令（-1.0 ~ 1.0）
- `state`：状态切换（`getup`, `locomotion`, `getdown`, `passive`）

## 使用方法

### 1. 启动 MuJoCo 仿真（带 UDP）

```bash
cd /home/dell/Github/rl_sar
./cmake_build/bin/rl_sim_mujoco 0315 scene_0315
```

启动后会在日志中看到：
```
[UDP] Remote control server started on port 9876
```

### 2. 启动 Flask 桥接服务器

```bash
python3 scripts/udp_remote_bridge.py
```

默认桥接到本机 (`127.0.0.1:9876`)。如果仿真在另一台机器上：
```bash
python3 scripts/udp_remote_bridge.py 192.168.x.x
```

### 3. 手机访问

确保手机和运行 Flask 的机器在同一局域网，然后浏览器打开：
```
http://<运行Flask的机器IP>:5000/
```

例如：`http://192.168.1.100:5000/`

### 4. 界面说明

- **左摇杆**：控制 X（前后）/ Y（左右）平移速度
- **右摇杆**：控制 Yaw（旋转）速度
- **GetUp (0)**：站立
- **RL Run (1)**：启动 RL 运动控制
- **GetDown (9)**：趴下
- **Passive (P)**：被动模式（零力矩）

## 文件变更

| 文件 | 变更 |
|------|------|
| `src/rl_sar/include/rl_sim_mujoco.hpp` | 添加 UDP socket、线程、原子命令变量 |
| `src/rl_sar/src/rl_sim_mujoco.cpp` | UDP 服务器实现；`RobotControl()` 注入 UDP 命令 |
| `scripts/udp_remote_bridge.py` | Flask HTTP → UDP 桥接 + 移动端网页 |
