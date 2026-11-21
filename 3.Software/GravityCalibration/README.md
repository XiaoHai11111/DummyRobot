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
| 现有 MJCF 重力力矩预览、局部拟合电流预览 | 已实现，见 `gravity_preview.py`；只离线计算 |
| F1 ADC 实测相电流、同步快照协议 | 未实现，需要先核对具体板卡电路 |
| 全姿态物理参数辨识、F1 重力前馈、拖动控制模式 | 本阶段给出设计和验收方法，尚未接入实机控制 |

**当前“电流”是 F1 控制器输出的有符号电流指令 `focCurrent`，不是 ADC 实测电流。** 文件使用 `i_cmd_motor_a` 和 `current_source=command` 明确区分。没有实机标定数据，因此没有填入任何真实重力补偿参数；生成的模型默认 `enabled=false`。

## 先离线跑通

在仓库根目录运行；沿用已有环境，不需要更改 MuJoCo 子模块：

```bash
conda activate DummyRobot
cd 3.Software/GravityCalibration
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v

python demo_data.py
python fit_local.py --joint 2 --current-limit-a 3.0 \
  --train data/synthetic/train.csv --validate data/synthetic/validation.csv \
  --output results/synthetic_j2.json
python gravity_preview.py --local-model results/synthetic_j2.json --q-deg 0 0 90 0 0 0
python gravity_preview.py --q-deg 0 0 90 0 0 0
```

`synthetic_*` 是人工生成的数据和参数，只用于验证流程，不能部署。最后一条使用当前 MJCF 中的质量、质心和 `DummyWeb/config.json` 的坐标映射，输出六轴 N·m 重力力矩；它依赖已有环境中的 `mujoco`，不连接串口，不代表已经标定准确。

## 第一次实机采集

1. 阅读完整方案。确认安装、工具和线缆状态；核对当前 F1 已支持 `0xA1` 扩展状态，升级本次 F4。**本次没有烧录任何固件，也没有连接或运动实机。**
2. 复制配置：`cp config.example.json config.j2.p0.json`。例子中的姿态和范围只是格式示例，未经过你的工作台碰撞验证。改成已检查的姿态、运动范围、负载描述、温度记录和已烧录的固件版本；填写该轴实际配置的 `current_limit_a`，确认后设置 `operator_reviewed_workspace=true`。这里的限流值仅供采集保护和筛选使用，脚本不会改变 F1 限流。`3.0 A` 只出现在合成例子，不能视为实机推荐值。
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

结果包含条件数、训练/验证 RMSE、验证偏差和最大误差；另存 `.train.csv`、`.validation.csv`，列出每个角度配对后的重力电流、正反向损耗和速度。`C` 可能包括偏置和线缆作用，并非独立的物理重力参数。程序不根据低 RMSE 自动批准实机部署。

检查验证误差、覆盖范围、正反向损耗及跨周期重复性，再按完整方案做位置前馈实验。`calibration.py:predict_local` 会拒绝超出标定角度或固定姿态的数据；它适合验证单关节实验，不能直接覆盖六轴自由运动。

## 文件导航

| 文件 | 用途 |
|---|---|
| `docs/标定与补偿方案.md` | 代码现状、实验矩阵、数学模型、参数可辨识性、补偿接入与验收 |
| `docs/固件接口.md` | 已实现协议、时间戳和单位约定、后续 F1/F4 修改清单 |
| `collect.py` | 受限的单关节往复采集，默认离线预览 |
| `calibration.py` | 坐标换算、数据筛选、配对、拟合与限幅限速参考函数 |
| `fit_local.py` | 局部模型训练及独立验证，导出 JSON 和配对 CSV |
| `gravity_preview.py` | 计算局部电流或现有 MJCF 的重力力矩，不下发控制 |
| `demo_data.py` | 构造合成数据验证流程 |
| `tests/` | 单位、方向、配对拟合、拒绝无效数据和协议解析测试 |

从仓库根目录运行 `bash 6.Tests/firmware/run.sh` 检查固件主机回归测试；本次也通过了现有 F4 Debug 工程的 ARM 交叉编译。主机测试和编译不能替代实机时序、传感器及运动验证。
