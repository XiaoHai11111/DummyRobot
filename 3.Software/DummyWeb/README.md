# DummyWeb · DummyRobot 本地控制台

基于本地 Python HTTP 桥接服务和 HTML/Three.js 页面，提供 DummyRobot 串口控制、关节/夹爪滑块、ASCII 指令窗口、D435 实时预览、关节状态遥测，以及 DummyMujoco URDF/MJCF 网格模型显示。Three.js 与 STLLoader 已放在 `static/vendor/`，页面运行不依赖 CDN。

## 启动

项目使用你已有的 `DummyRobot` Conda 环境。先确认环境里有 `pyserial`、`numpy`、`opencv-python` 和 `pyrealsense2`；若有缺失，只在该环境内安装：

```bash
conda activate DummyRobot
python -m pip install -r /home/xiaohai/workspace/DummyRobot/3.Software/DummyWeb/requirements.txt
```

启动本机服务：

```bash
cd /home/xiaohai/workspace/DummyRobot/3.Software/DummyWeb
./run.sh
```

浏览器打开 `http://127.0.0.1:8765`。服务默认只绑定本机回环地址，关闭终端或按 `Ctrl+C` 停止。

## 页面操作

1. 点端口旁的扫描图标，选择主控虚拟串口（例如 `/dev/ttyACM0`）。按主控固件配置选择波特率、数据位、校验位和停止位；当前参数默认 115200、8N1。USB CDC 不依赖波特率时，仍保留该选项用于串口驱动配置。
2. 点“连接机械臂”。连接后会轮询 `#GETJPOS` 和 `#GETLPOS`，页面显示实际关节角和 TCP 姿态。
3. 点“使能”后可以拖动 J1–J6 滑块；拖动过程中按约 100 ms 节流持续发送目标关节角，松开时再兜底发送一次。所有关节目标命令统一使用 `&j1,j2,j3,j4,j5,j6,speed`，速度从 `config.json` 读取，默认 **100°/s**。
4. 夹爪可用滑块精调，也可以点“打开夹爪”或“关闭夹爪”；协议发送 `#GRIP angle`，范围默认 0–45°。现有固件没有夹爪角度回读接口，页面模型显示的是最后发出的夹爪目标角，不代表夹爪测得的实际位置。
5. “Homing”与“Resting”使用关节控制命令移动到配置文件中的姿态，不依赖固件增加新命令。默认姿态分别为 `[0, 0, 90, 0, 0, 0]` 和 `[0, -75, 180, 0, 0, 0]`。点击后会先弹出确认框；执行期间按钮显示 `Resetting…` / `Returning…`，到达目标或超时后恢复。
6. 点“连接相机”启动 D435 彩色/深度流，页面显示彩色预览；断开按钮停止相机流。
7. 页面中间自动加载 `dummy.xml`（MJCF）数字孪生，机器人实际关节反馈通过配置里的模型关节映射驱动 Three.js 模型。
8. 在“串口指令”面板输入一条 ASCII 命令，点击“发送”或按回车。面板显示发送内容、固件回包和错误。运动、使能、夹爪和模式设置会要求二次确认；`!STOP` 可直接发送。指令必须属于下表支持的格式，未列出的命令会被拒绝。

连接状态和使能状态在启动时分开处理。当前 ASCII 协议没有读取“机械臂是否已使能”的命令，因此页面只显示本页面发出的使能/失能状态；若机械臂在其他程序中改变使能状态，重新连接后应先在页面执行一次明确的使能或失能操作。断开串口不会自动改变机械臂使能状态。

## 串口指令窗口与固件 ASCII 命令

页面指令窗口通过 USB 虚拟串口发送一条命令。后端只允许下表列出的指令，并会校验参数；它不是任意串口透传终端。

| 功能 | 命令格式 | 参数与行为 |
|---|---|---|
| 读取关节角 | `#GETJPOS` | 返回 `okj J1 J2 J3 J4 J5 J6`，角度单位为度 |
| 读取末端位姿 | `#GETLPOS` | 返回 `okl X Y Z A B C`；XYZ 为毫米，ABC 为度 |
| 使能机械臂 | `!START` | 使能电机；页面会二次确认 |
| 失能机械臂 | `!DISABLE` | 失能电机 |
| 急停 | `!STOP` | 请求固件停止当前运动；页面可直接发送。急停后仍需检查实机状态 |
| 设置命令模式 | `#CMDMODE mode` | `1` 顺序目标点；`2` 可中断目标点（固件默认）；`3` 电机调试。固件此命令不返回确认文本 |
| 设置夹爪角 | `#GRIP angle` | `angle` 范围为 `0` 到 `45` 度；页面要求已使能并二次确认 |
| 设置夹爪电流上限 | `#GRIP_CURRENT current` | `current` 为 `0` 到 `1` 的归一化值；页面要求已使能并二次确认 |
| 关节目标（直接控制） | `&j1,j2,j3,j4,j5,j6,speed` | 关节角单位为度；速度必须等于 `config.json` 中的 `robot.joint_speed_deg_s`（当前为 **100°/s**） |
| 关节目标（命令队列） | `>j1,j2,j3,j4,j5,j6,speed` | 速度必须为配置值。模式 1 按顺序执行；模式 2 可中断当前目标 |
| 直线末端目标 | `@x,y,z,a,b,c,speed` | XYZ 为毫米，ABC 为度；速度必须为配置值。目标会进入固件运动队列 |

`&` 命令还允许两种固件扩展格式：`&j1,j2,j3,j4,j5,j6,speed,gripper_angle`，以及再追加一个 `gripper_current` 的九参数格式。追加的夹爪角范围为 `0–45°`，电流范围为 `0–1`。网页会对六关节角、夹爪角、电流和速度做校验。

运动命令与夹爪动作需要机械臂先连接、使能并通过页面确认。命令回包可能表示“已进入队列”或“命令已接收”，并不保证机械臂已经到位；请结合实时关节/TCP 遥测确认运动结果。`#CMDMODE 3` 是电机调试模式，普通 `>` / `@` 队列运动不适用于该模式。

Homing / Resting 不是 USB ASCII 命令，而是页面通过六关节目标实现的两个预设动作，也不是限位开关寻零。`reboot`、Fibre CLI 对象方法、调试接口和未列出的 ASCII 命令不能通过此窗口发送。请先确认机械臂周围空间、关节限位、速度和目标姿态适合实机，再使能并运动。

## 重要关节位置

```bash
# Homing 7字型
&0, 0, 90, 0, 0, 0, 100

# Reseting
&0, -75, 180, 0, 0, 0 ,100

# 抓取前图像采集
&-12, -43, 125, 0, 86, 0, 100
```



## 可调参数

固定参数文件为 [`config.json`](config.json)，修改后重启服务生效：

- `server.host`、`server.port`：本地服务地址与端口；建议保持 `127.0.0.1`。
- `serial.*`：默认端口、115200 波特率、8 数据位、N 校验、1 停止位、读写超时、轮询频率。默认 100 ms（10 Hz），页面会限制并发状态请求，避免串口轮询堆积。
- `robot.joint_speed_deg_s`：所有关节目标命令发送的速度；默认 100°/s。
- `robot.joints[].min_deg/max_deg`：滑块和后端的关节范围，取自当前固件各电机角度限位。
- `robot.joints[].model_joint/model_sign/model_offset_deg`：实机轴到 URDF/MJCF 关节的映射、方向和零位偏置。当前 J3 映射偏置设为 -90°，按实际模型与实机关节方向核对后可调整。
- `robot.homing_pose_deg`、`robot.resting_pose_deg`：两个关节空间预设目标。
- `robot.gripper.*`：夹爪角范围、开/合角和模型手指关节映射。
- `camera.*`：彩色/深度分辨率、帧率和预览 JPEG 质量。
- `models.*`：DummyMujoco 中 URDF、MJCF 和 STL 网格目录。

页面里的波特率/帧格式是连接时选择的参数；关节速度及关节映射由此文件统一维护。

## 文件结构

```text
DummyWeb/
├── config.json
├── server.py
├── run.sh
└── static/
    ├── index.html
    ├── style.css
    ├── app.js
    └── vendor/       # 本地 Three.js / OrbitControls / STLLoader
```

如果主控固件更换了串口协议，需要同步修改 `server.py` 中的命令解析和 ASCII 命令格式。
