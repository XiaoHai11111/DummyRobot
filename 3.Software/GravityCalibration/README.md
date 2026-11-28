# DummyRobot 参数识别与重力补偿

目标：**先改善位置控制与保持，再实现拖动示教**。推荐从你提出的“其他关节保持、单关节低速往复”开始，先标定重力对应的电流和摩擦，再扩展多姿态重力模型。低速实验不能完整识别惯量、科氏项，也不能在未知电流—力矩比例时直接得到可信的质量和质心。

完整实验和落地设计见 [标定与补偿方案](docs/标定与补偿方案.md)，通信与后续固件任务见 [固件接口](docs/固件接口.md)。

## 这次已经提供的内容

| 内容 | 状态 |
|---|---|
| F1 `0x21/0x22/0x23` 电流、速度、位置查询 | 原代码已有；本次复用 |
| F4 接收电流、速度，`#GETTELEM 1..6` 导出带时间戳的读数 | 已实现；需要将更新后的 F4 固件烧入实机 |
| 单关节往复采集 `collect.py` | 已实现；默认只预览，`--execute` 才启用并移动实机 |
| 正反向配对、局部重力电流拟合、独立验证集 | 已实现，见 `fit_local.py` |
| J4/J5 多姿态 MJCF 重力、一转纹波和方向摩擦拟合 | 已实现，见 `fit_wrist.py`；候选模型默认禁用 |
| J4/J5 模式 5 拖动验收离线预演与实机记录 | 已实现，见 `wrist_drag_trial.py`；默认离线，`--execute` 才实机 |
| 现有 MJCF 重力力矩预览、局部拟合电流预览 | 已实现，见 `gravity_preview.py`；只离线计算 |
| F1 ADC 实测相电流、同步快照协议 | 未实现，需要先核对具体板卡电路 |
| 全姿态物理参数辨识、F1 重力前馈、拖动控制模式 | 本阶段给出设计和验收方法，尚未接入实机控制 |

**当前“电流”是 F1 控制器输出的有符号电流指令 `focCurrent`，不是 ADC 实测电流。** 文件使用 `i_cmd_motor_a` 和 `current_source=command` 明确区分。现有实测只用于离线候选拟合，所有生成模型仍为 `enabled=false`。

## 先离线跑通

在仓库根目录运行；沿用已有环境和指定的解释器，不需要更改 MuJoCo 子模块：

```bash
PYTHON=/home/xiaohai/miniconda3/envs/DummyRobot/bin/python
cd 3.Software/GravityCalibration
$PYTHON -m pip install -r requirements.txt
$PYTHON -m unittest discover -s tests -v

$PYTHON demo_data.py
$PYTHON fit_local.py --joint 2 --current-limit-a 3.0 \
  --train data/synthetic/train.csv --validate data/synthetic/validation.csv \
  --output results/synthetic_j2.json
$PYTHON gravity_preview.py --local-model results/synthetic_j2.json --q-deg 0 0 90 0 0 0
$PYTHON gravity_preview.py --q-deg 0 0 90 0 0 0
```

`synthetic_*` 是人工生成的数据和参数，只用于验证流程，不能部署。最后一条使用当前 MJCF 中的质量、质心和 `DummyWeb/config.json` 的坐标映射，输出六轴 N·m 重力力矩；它依赖已有环境中的 `mujoco`，不连接串口，不代表已经标定准确。

## 第一次实机采集

1. 阅读完整方案。确认安装、工具和线缆状态；核对当前 F1 已支持 `0xA1` 扩展状态，升级本次 F4。**本次没有烧录任何固件，也没有连接或运动实机。**
2. 复制配置：`cp config.example.json config.j2.p0.json`。例子中的姿态和范围只是格式示例，未经过你的工作台碰撞验证。改成已检查的姿态、运动范围、负载描述、夹爪开度、温度记录和已烧录的固件版本；填写该轴实际配置的 `current_limit_a`，确认后设置 `operator_reviewed_workspace=true`。`gripper_rad` 使用 MJCF 关节角，闭合为 `0`，全开为 `0.78`。这里的限流值仅供采集保护和筛选使用，脚本不会改变 F1 限流。`3.0 A` 只出现在合成例子，不能视为实机推荐值。
3. 用现有界面将机械臂移动至该姿态，使目标关节位于扫描区间内。其他关节与配置误差需小于 0.5°。使用模式 2（可中断位置模式），保留现有闭环参数和过载保护。脚本不会主动切模式，因为当前固件的 `#CMDMODE 2` 会重设加速度；若需要切换，应在整组实验前完成并记录。关闭 DummyWeb/CLI 等串口客户端，再运行脚本。
4. 首次建议只做一个窄范围往复、1°/s、20 Hz，检查曲线和方向。通过后扩大已验证的范围，做 3–5 个周期及多个速度。脚本的默认 4 周期配置适用于之后的正式采集。

```bash
# 不打开串口，不移动机械臂
python collect.py --config config.j2.p0.json

# 明确执行实机运动：清掉旧运动、校验姿态、启用、低速往复并记录
python collect.py --config config.j2.p0.json --execute \
  --port /dev/ttyACM0 --output data/j2_p0_v1_train.csv

# 独立再采一整轮，作为验证集；配置/负载/固定姿态保持一致
python collect.py --config config.j2.p0.json --execute \
  --port /dev/ttyACM0 --output data/j2_p0_v1_validation.csv
```

运行期间 `Ctrl+C`、超时、故障、其他关节漂移、接近已填写限流值或过大时间偏差都会退出，并尝试 `!STOP` 保持当前位置；不会 `!DISABLE` 让承重轴失去支撑，也不会自动清错。**USB 断连或进程被强杀时无法保证 `!STOP` 送达**，执行时要保持独立急停可用。这是当前控制系统缺少“上位机运动租约”的实际限制，详见方案。

输出包含 CSV 和同名 `.meta.json`（配置、是否完成、是否收到停止确认）。CSV 不覆盖已有文件。`q1..q6_deg` 记录全部实际关节角；活动轴同时记录电流、速度和各自接收时间，其他轴角度来自 F4 缓存，不能当作六轴同步动态测量。

## J4/J5 国产减速器基准采集

`pose_sweep_planner.py` 会读取当前 MJCF (`DummyMujoco/dummy_description/mjcf/scene.xml`) 和 `DummyWeb/config.json` 的角度映射，在腕部历史采集范围的交集内组合姿态。严格腕部交集为 J1 `[-155,155]`、J2 `[-70,55]`、J3 `[40,165]`、J4 `[-170,170]`、J5 `[-100,100]`、J6 `[-360,360]` 度。J1、J2、J3、J6 仍可按传统配置采集。J4/J5 的旧配置可以离线预览和拟合，但 `collect.py --execute` 或 `batch_collect.py --execute` 缺少 `safety_profile=wrist_strict_v1` 时会明确拒绝，避免旧 J4/J5 配置绕过历史限位、当前 MJCF、地面碰撞和夹爪检查。严格实机配置还必须用实际 F1/F4 构建填写 64 位十六进制 `firmware_hash`。

- TCP 坐标：`x > -200 mm`、`z > 5 mm`，姿态不限制。
- 沿转场和扫描路径每 `1°` 做一次 MuJoCo 正运动学和碰撞检查。所有非相邻连杆的自碰撞和地面碰撞都严格检查，接触深度最多允许 `1e-9 m` 的浮点误差；闭合夹爪两指网格约 `3e-10 m` 的数值接触可通过。`base_link/upper_link` 约 `0.95 mm` 的穿模不再豁免，MJCF 中出现该接触即拒绝。执行入口没有毫米级碰撞容差参数。
- 规划和执行使用配置中的同一 `gripper_rad` 计算夹爪几何。生成配置必须显式记录该开度，严格采集缺少或越界时拒绝。
- 批次内活动轴可以从扫描区间任意起点开始。每一段入队前都会重新读取六轴，先核对实际起点与已审查路线；若与计划偏差超限，或上一段 `!STOP` 未确认，下一次使能会被拒绝。
- 每个关节最多选 8 个尽量分散的固定姿态，候选必须先通过参考来源转场、固定姿态扫描和姿态间安全接力三段检查，再按实际安全顺序输出配置。当前严格 MJCF 下 J4、J5 各找到 7 个完整可用姿态，共生成 42 个速度配置；统一 `50 Hz`、`5` 个正反向周期。
- 候选姿态始终包含已验证可用的 `[-90,-16,110,0,0,0]°`，选源网格同时包含 J1 `±90°`、J2 `-16°` 和 J3 `110°`，避免现有数据无法提供实际来源、转场或扫描姿态。
- 新生成的配置将 `operator_reviewed_workspace` 写为 `false`，现场核对夹具、线缆和工作台后才能手工改为 `true`。

```bash
# 写入新的输出目录，不连接串口、不运动实机
$PYTHON pose_sweep_planner.py --output-dir /tmp/j45_plan

# 离线逐个预览命令，不打开串口
$PYTHON batch_collect.py --config-dir /tmp/j45_plan --joint 4 --limit 3

# 实机执行；执行前仍需人工核对物理工作台，保持独立急停可用
$PYTHON batch_collect.py --config-dir /tmp/j45_plan --execute --port /dev/ttyACM0

# 中断后续采，只跳过 completed=true 且 stop_acknowledged=true 的配置
$PYTHON batch_collect.py --config-dir /tmp/j45_plan --resume --execute --port /dev/ttyACM0
```

规划和执行入口不会覆盖已有配置、CSV、元数据或通信日志。规划阶段不采用只扫得通、却从参考姿态开不过去的候选。批执行先用六次新鲜的 `#GETTELEM` 读取实际姿态，确认状态正常、模式为 2 且尚未使能，再按 J1、J2、J3、J6、J4、J5 的实际转场顺序预检完整路线，并从固定姿态继续预检 `low -> high -> low` 的全部往返段。任一无效段都会在首次 `!START` 前终止。开始后每个分段在使能后、入队前再次完整读取六轴并复核，运行中以 10 Hz 重读全部六轴，在线检查 TCP、碰撞、故障、模式、遥测和关节空间路线偏离；偏离路径即退出并确认 `!STOP`。

`collect.py` 只对 F4 返回的 `F1 telemetry timeout` 做有限重试，并忽略迟到的遥测错误行；其他故障仍立即中止。活动轴按 `sample_hz` 读取，其他轴和 TCP 默认另以 `full_pose_poll_hz=2` 完整轮询，并在每段入队前再完整读取六轴，避免方向保持时长期依赖 F4 缓存。J4 节点当前约半数遥测请求会超时，配置仍写 `50 Hz`，但有效采样率会低于 50 Hz，必须以 CSV 时间戳和 `.meta.json` 中的 `comm_retries` 评估。`pose_summary.json` 记录每个姿态的扫描范围、TCP 最小 x/z、夹爪开度和来源。收尾 `!STOP` 未确认时，批定位和采集入口都抛出错误，元数据不会标为完成；若采集主体已有异常，错误信息同时保留两条失败原因。MJCF 通过不代表真实工作台无干涉，`operator_reviewed_workspace=true` 也不能替代现场确认。

### 已验证六路线补采

`wrist_supplement_plan.py` 直接使用内置 `EXPECTED_ROUTES` 和公共起点 `[-90,-16,110,0,0,0]°` 定义六条补采路线，不读取 `results/` 内的历史证据。六条路线分别固定 J5 为 `-60/0/+60°` 扫描 J4，或固定 J4 为 `-120/0/+120°` 扫描 J5，扫描区均为 `[-40,40]°`。每次生成都会用当前 MJCF 按实际批次顺序离线复验转场和往返。主采包含 `0.5/1/2°/s`，另有带 `measurement_round=independent_retest` 标签的 `1°/s` 复测轮，共 24 份配置。`--verified-routes` 仅用于可选的历史证据一致性比对。

```bash
# 只在不存在的新目录中生成，重新读取当前 MJCF 并离线复验全部转场和往返
$PYTHON wrist_supplement_plan.py \
  --current-limit-a 1.0 \
  --output-dir results/pose_configs_j45_supplement_20261127

# 同时绑定两个实际固件 HEX，记录各自 SHA-256 和确定性组合标记
$PYTHON wrist_supplement_plan.py \
  --current-limit-a 1.0 \
  --output-dir /tmp/j45_supplement_with_firmware \
  --f1-hex ../../2.Firmware/Ctrl-Step-Driver-STM32F1-fw/cmake-build-wrist-v2/Ctrl-Step-STM32-fw.hex \
  --f4-hex ../../2.Firmware/Core-STM32F4-fw/cmake-build-wrist-v2/Core-STM32F4-fw.hex

# 离线逐个检查，不打开串口
$PYTHON batch_collect.py \
  --config-dir results/pose_configs_j45_supplement_20261127

# 操作者按现场填写当前限流、固件 revision/hash 并设 operator_reviewed_workspace=true 后
$PYTHON batch_collect.py \
  --config-dir results/pose_configs_j45_supplement_20261127 \
  --output-dir data/j45_supplement_20261127 \
  --resume --execute --port /dev/ttyACM0
```

两个 HEX 必须同时提供。配置继续保留现有 `firmware_revision` 和 64 位 `firmware_hash` 字段，其中 `firmware_hash` 使用按 `{F1_HEX,F4_HEX}` 规范 JSON 计算的组合 SHA-256，并另存 `firmware_artifacts` 中的单文件 SHA-256。未提供 HEX 时仍可生成 24 份待复核配置。生成配置的夹爪为闭合 `0 rad`，无夹持物，相机已安装且数据线接着。`operator_reviewed_workspace` 保持 `false`，固件 revision 和限流来源仍需现场确认，严格执行入口会拒绝占位值。工具不会自动连接或运动，也不会覆盖旧配置与数据目录。

## 拖动验收离线预演（wrist_drag_trial.py）

`wrist_drag_trial.py` 默认只做离线预演：读取当前 MJCF 和 `DummyWeb/config.json` 的角度映射，按参考姿态、腕部区间、其他轴偏差和夹爪开度生成并复验 J4/J5 拖动走廊，打印 JSON，不打开串口、不运动。仅 `--execute` 才打开串口，并依次确认停止的模式 2、使能、切到 DRAG 模式 5、记录样本、收尾 `!STOP` 并恢复模式 2；它从不写关节目标，也不写固件/EEPROM 参数。

离线预演示例（不连接实机）：

```bash
$PYTHON wrist_drag_trial.py \
  --joint 4 \
  --reference -90 -16 110 -25 -60 0 \
  --j4-min -38 --j4-max -12 \
  --j5-min -61 --j5-max -59 \
  --other-deviation 1 \
  --duration 18 \
  --gripper-rad 0 \
  --corridor-step .25 \
  --hands-off
```

输出 `serial_opened=false` 和 `corridor_preflight`（检查姿态数、TCP 最小 x/z、严格腕限位等），用于确认该关节姿态在当前 MJCF 下可离线通过。

真实试验必须同时给出 `--execute`、`--port` 和 `--output`，并在 `--hands-off`、`--keyboard-release`、`--release-marker-file` 中恰好选一种：

- `--hands-off`：记录一段独立 10 秒静置，计时从进入 DRAG 模式后的首个采样开始；不是操作者真实松手标记。
- `--keyboard-release`：运行中按回车表示真实松手。
- `--release-marker-file PATH`：以文件出现表示真实松手。

验收指标来自 F4 position tick：脚本取 `POST_RELEASE_S=10.0` 的窗口计算 `net_drift_deg`、`peak_to_peak_deg` 和 `max_tick_gap_ms`，并写进 `.meta.json`；`--hands-off` 的窗口从进入 DRAG 模式后的首个采样开始，`--keyboard-release` 与 `--release-marker-file` 从对应松手标记采样开始。CSV 同时记录 J4/J5 指令电流、位置 tick、活动轴速度和 `#GETWRIST` 回读。这些是编码器/时间戳客观指标，不是操作者手感；手感需另记，不能从 CSV 推断。

同一参考姿态与区间做 A/B 比较时，两次运行必须复用完全相同的 `--reference`、`--j4-min/--j4-max`、`--j5-min/--j5-max` 和 `--gripper-rad`，只改变待比较的配置（例如 gain/cap），并用不同输出文件名区分；脚本本身不自动标注 A/B。

外部条件需现场确认并保持一致：夹爪闭合空载（`--gripper-rad 0`）、相机已安装且数据线接着、工作台与线缆已检查、独立急停可用；`--execute` 会从 `#GETDRAG` 读回拖动限流并要求其在 `0.1..1.5 A`，否则中止。

## 拟合与评估

下面的 `实际限流值` 必须替换为该关节的真实设置，单位 A：

```bash
python fit_local.py --joint 2 --current-limit-a 实际限流值 \
  --train data/j2_p0_v1_train.csv \
  --validate data/j2_p0_v1_validation.csv \
  --output results/j2_p0.json
```

同一固定姿态的多个速度文件可以一起传给 `--train`；另行采集的完整实验传给 `--validate`。不同固定姿态不能混入一个局部模型。模型为：

```text
i_g_joint(q) ≈ A*sin(q_rad) + B*cos(q_rad) + C
i_g_motor = motor_sign[j] * i_g_joint
```

结果包含条件数、训练/验证 RMSE、验证偏差和最大误差；另存 `.train.csv`、`.validation.csv`，保持传统四列契约 `q_deg,gravity_current_a,directional_loss_a,speed_deg_s`。第四列是正反向实际速度幅值的平均值，不是带符号速度。`C` 可能包括偏置和线缆作用，并非独立的物理重力参数。程序不根据低 RMSE 自动批准实机部署。

检查验证误差、覆盖范围、正反向损耗及跨周期重复性，再按完整方案做位置前馈实验。`calibration.py:predict_local` 会拒绝超出标定角度或固定姿态的数据；它适合验证单关节实验，不能直接覆盖六轴自由运动。

## J4/J5 离线候选模型

默认命令以 `fit_wrist.py` 所在目录为数据根，只发现同时带 `completed=true` 和 `stop_acknowledged=true` 元数据的 J4/J5 CSV，再按固件与载荷分组。载荷名中的 `J4=<角度>` 姿态备注会被归一化，原始描述和文件哈希保留在输入清单。这个通用示例会扫描默认全项目数据，可能混入其他批次，只用于确认流程；本次拟合必须显式限定数据目录和输出目录。

```bash
$PYTHON fit_wrist.py

$PYTHON fit_wrist.py \
  --data-root data/wrist_v2_onsite_20261128_verified \
  --output-dir results/wrist_v2_onsite_20261128_fit \
  --time-source f4-position
```

第一条是保留的通用示例，默认输出位于 `results/j45_offline_fit_candidates_v2/`；第二条使用本次现场已核验数据，输出位于 `results/wrist_v2_onsite_20261128_fit/`。两条命令的时间源不同：第一条面向历史数据，沿用默认 host 时间；第二条面向本次现场采集，样本时间以 F4 position tick 为准，避免 host 接收时间受 USB/串口调度延迟影响，因此显式保留 `--time-source f4-position`。对应补充批次为 24 份配置（18 主采 + 6 整轮复测）。脚本使用加权最小二乘，让每个独立轮总权重相同，并同时导出整姿态留出和独立轮留出。候选公式为：

`measurement_round=independent_retest` 的轮次会在分组后移出训练集。训练权重、速度项选择、整姿态留出和独立轮留出全部只使用未标记数据。候选系数冻结后才对复测轮计算 E/O RMSE，并单独写入 `independent_retest`。没有该标签的历史数据仍按现有留姿态和留轮逻辑处理。

```text
E = k*tau_hold_Nm + bias + E_sin*sin(30*q_robot_rad) + E_cos*cos(30*q_robot_rad)
O = odd_constant + odd_speed*abs(speed_deg_s)
    + O_sin*sin(30*q_robot_rad) + O_cos*cos(30*q_robot_rad)
i_joint_positive = E + direction*O
i_motor_joint_phase = motor_sign[j]*i_joint_positive
```

`tau_hold_Nm` 是当前闭合 MJCF 的零速关节保持力矩。`q_robot_deg` 直接取 F4 `#GETJPOS` 返回值，J4/J5 不再叠加 `initPose`。`direction=+1` 对应关节正方向运动，`-1` 对应反方向。速度辨识以元数据中的真实指令速度分层，要求至少两个有实际跨度的指令速度；每个留出折只在训练数据内决定是否加入粘滞项，再用于测试折。该项还要求正号、95% 置信区间不跨零且 BIC 至少改善 2，否则导出 `included=false` 和拒绝原因；负粘滞系数不会进入可部署公式。

`model.motor` 仍明确标注为“关节角相位 `phi` + 关节运动方向 `dir_joint` 后再乘 motor 符号”的中间表达，不能直接当作 F1 电机角参数。真实 F1 相位为 `psi = motor_sign_J * 30*q_robot_rad`，电机运动方向为 `dir_motor = motor_sign_J * dir_joint`。模型另导出 `motor_phase`：在 gain=100% 时的未缩放四谐波为 `(E_sin, E_cos*s, O_sin*s, O_cos)` mA，J4 `s=+1`、J5 `s=-1`；库仑项单位为 mA，粘滞项单位为 mA/(motor rev/s)，均给出“最近整数、正好半值远离零”的舍入结果。J5 只有单一速度的数据不能部署粘滞项。

每个模型包含关节正方向的 E/O 系数、转换到 F1 motor 方向的系数与置信区间、覆盖的姿态/角度/力矩/速度、留出误差、输入文件哈希和限制。J4 的 motor 符号为 `+1`，J5 为 `-1`。固件 revision 与实际 hash、显式 `payload_identity`、夹爪开度共同分组；新增连线相机与拔线相机配置不会混为同组。MJCF 重力按每组实际 `gripper_rad` 计算。所有候选保持 `enabled=false` 和 `deployment_review_required=true`。输出目录带清单，默认重跑可复现同一结果。清单更新不再删除旧清单中的任何路径，避免旧版本把输入 CSV 误当作过期输出清理。

## 文件导航

| 文件 | 用途 |
|---|---|
| `docs/标定与补偿方案.md` | 代码现状、实验矩阵、数学模型、参数可辨识性、补偿接入与验收 |
| `docs/固件接口.md` | 已实现协议、时间戳和单位约定、后续 F1/F4 修改清单 |
| `collect.py` | 受限的单关节往复采集，默认离线预览 |
| `calibration.py` | 坐标换算、数据筛选、配对、拟合与限幅限速参考函数 |
| `fit_local.py` | 局部模型训练及独立验证，导出 JSON 和配对 CSV |
| `fit_wrist.py` | J4/J5 多姿态 MJCF 重力、纹波和方向摩擦候选拟合，默认离线 |
| `wrist_drag_trial.py` | J4/J5 模式 5 拖动验收窗口，默认离线预演，`--execute` 实机记录静置或松手标记后 10 s 漂移 |
| `gravity_preview.py` | 计算局部电流或现有 MJCF 的重力力矩，不下发控制 |
| `pose_sweep_planner.py` | 基于 MJCF 筛选 J4/J5 姿态并生成采集配置 |
| `wrist_supplement_plan.py` | 从内置六路线和当前 MJCF 生成主采与独立复测配置，只做离线预演 |
| `batch_collect.py` | 按生成配置逐个调用 `collect.py`，默认不连接实机 |
| `demo_data.py` | 构造合成数据验证流程 |
| `tests/` | 单位、方向、配对拟合、拒绝无效数据和协议解析测试 |

从仓库根目录运行 `bash 6.Tests/firmware/run.sh` 检查固件主机回归测试。本目录离线测试和固件主机测试都不能替代实机时序、传感器、碰撞及运动验证。截至 2026-11-28，标定目录 76 个 Python 测试通过，`bash 6.Tests/firmware/run.sh` 通过；历史版本曾在 `test_f4.cpp:236 VerifyReferenceGravity` 处以 `2.72885 N·m` 最大误差失败，现已被当前固件回归覆盖。单独 MJCF 核验脚本 `python3 6.Tests/firmware/test_wrist_gravity.py` 对 65 姿态的最大误差为 `2.546e-7 N·m`（阈值 `1e-5 N·m`）。
