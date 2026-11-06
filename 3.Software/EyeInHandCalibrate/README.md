# 项目说明

本项目基于 https://github.com/leo038/hand_eye_calibrate.git 制作；

手眼标定的原理可参考：[机械臂手眼标定方法详解](https://blog.csdn.net/leo0308/article/details/141498200)；

基于源项目编写了眼在手外与眼在手内的标定程序。UR5 示例采集器仍保留；`hand_in/data_collect_in.py` 已适配 DummyRobot 主控的 USB 虚拟串口。

需要自行安装Intel D435的驱动，安装方法详见百度。

# 使用说明

## DummyRobot 眼在手上标定

标定板标签写的是 **8×11 个方格，方格边长 15 mm**。OpenCV 要求填写内部交点数量，所以程序使用 **7×10 个内角点**，格长为 **0.015 m**。这些参数集中在 `hand_in/calibration_settings.py`。

### 1. 准备环境和硬件

1. 在仓库根目录创建并启用项目环境（Miniconda 已安装时）：

   ```bash
   conda env create -f 3.Software/EyeInHandCalibrate/environment.yml
   conda activate DummyRobot
   ```

   环境已创建过时，之后只需运行 `conda activate DummyRobot`。CLion 中将 Python 解释器设为 `/home/xiaohai/miniconda3/envs/DummyRobot/bin/python`。环境包含 `numpy`、`opencv-python`、`pyrealsense2` 和 `pyserial`。
2. 将相机固定在机械臂末端，整个采集过程中不要松动相机或改变相机与末端之间的安装关系。
3. 将棋盘平整固定在桌面或支架上，标定期间棋盘不能移动；棋盘不要拿在手上跟着机械臂动。
4. 确认主控运行包含 `#GETLPOS` 的固件，USB 线连接到主控虚拟串口。用 `ls /dev/ttyACM*` 查 Linux 端口；有多个设备时明确选择主控端口。
5. 清理机械臂周边空间，确认急停可用。采集只读取位姿，不会由脚本命令机械臂运动；每次移动由操作者手动完成。

### 2. 采集图像和末端位姿

在终端运行：

```bash
cd 3.Software/EyeInHandCalibrate/hand_in
python data_collect_in.py --port /dev/ttyACM0
```

Windows 可将端口替换为 `COM5` 等设备名。省略 `--port` 时，程序按 USB VID `0x1209` 自动查找；也可通过环境变量 `DUMMYROBOT_PORT` 指定端口。程序先确认串口能读到位姿，然后打开相机预览。

1. 先把棋盘放在相机视野中央，调节距离、焦距/曝光和照明，让整个棋盘清晰可见。
2. 观察预览窗口：绿色角点覆盖并显示 `Board detected` 后，才按 `s` 保存；红色未识别时不要强行采集。被手、夹具遮住、反光、运动模糊、棋盘太小或超出画面都会影响识别。
3. 每次采集前先让机械臂停稳，再按 `s`。程序同时保存当前图像和从 `#GETLPOS` 读取的末端位姿。固件返回毫米和度，程序转换成米和弧度写入 `collect_data_in_dummyrobot/poses.txt`。
4. 采集 **20–30 组有效姿态**。每组都要看清完整棋盘，并改变相机相对于棋盘的姿态：覆盖画面中央、四周和不同距离；让棋盘有不同倾斜角度；机械臂绕至少两个不同轴改变末端方向。只平移相机或只绕一个轴转动会使结果不稳定。保持运动幅度在安全范围内，不要让棋盘离开相机视野。
5. 按 `q` 或 `Esc` 结束。新数据放在 `hand_in/collect_data_in_dummyrobot/`，与仓库已有 UR5 示例数据分开。

### 3. 检查采集数据并运行标定

1. 检查 `collect_data_in_dummyrobot/` 中图像数量与 `poses.txt` 行数一致；每行必须有 6 个数。可以抽查图像，确认棋盘完整、清晰且角点覆盖画面不同区域。
2. 从 `hand_in` 目录运行：

```bash
python hand_in_eye_calibrate.py
```

程序会逐张重新识别 7×10 个内角点，先计算相机内参，再计算相机相对于机械臂末端的旋转矩阵和平移向量。有效图像少于 10 张或有效图像数与位姿数不相等时会报错；建议保留 20–30 张有效图像。

结果保存在 `hand_in/measured_data_in_dummyrobot/`：`mtx.txt` 和 `dist.txt` 是相机内参/畸变；`rotation_matrix.txt`、`translation_vector.txt` 和 `camera_pose.txt` 是相机相对于末端的外参。平移向量单位为米。后续把这个变换用于机器人坐标换算时，先确认使用的是相机到末端还是末端到相机；若程序需要反向变换，应对齐次矩阵求逆。

眼在手外仍使用 `hand_out` 的程序；UR5 的手眼采集脚本保留为 `data_collect_in_ur5.py` / `data_collect_out_ur5.py`。

手眼标定数据会自动保存于各自的measured_data文件夹中，自行取用；

手眼标定结束后运行各文件夹中的record_realsense.py获取相机深度数据，所获取的深度数据也会自动保存于各自的measured_data文件夹中;

在手眼标定结束后可进入Acc_test文件夹，修改main.py中的各项数据，运行main.py进行手眼标定结果的验证；此验证仅适用于UR5与D435相机结合的项目，如果需要测试自己的项目，请自行替换相机文件（realsenseD435.py）与机器人文件（UR_Robot.py）。

# Tips

collect_data_in和collect_data_out文件夹中都有示例图片，可以直接运行理解代码。

# 联系方式

如有疑问请联系wangjie03690@163.com


