#!/usr/bin/env python3
# coding=utf-8
"""DummyRobot 版手眼标定精度验证（眼在手上）。

与 UR5 版 ``Acc_test/main.py`` 的差异：
- 机器人由 ``DummyRobotVCP``（主控 USB 虚拟串口 ASCII 协议）驱动，不再依赖 UR5 TCP 30003；
- 坐标换算遵循眼在手上的正确链式关系：

      p_base = T_base_end @ T_end_camera @ p_camera

  其中 ``T_base_end`` 来自主控 ``#GETLPOS``（实时末端位姿），
  ``T_end_camera`` 来自 ``hand_in`` 标定输出的 ``rotation_matrix.txt`` /
  ``translation_vector.txt``；
- 深度使用对齐到彩色流的 D435 帧，并按标定得到的 ``mtx.txt`` / ``dist.txt``
  做去畸变反投影，保证与标定阶段使用的相机模型一致。

三种用法：
1. 干运行（默认，不发送任何运动）：点击图像 → 打印相机系/基座系坐标。
   从不同机械臂位姿点击同一个物理点，再按 ``p`` 可得到这些估计基座坐标的
   一致性误差（这是手眼外参本身的量化指标）。
2. ``--execute``：点击后仍需逐个确认，才使能并 MoveL 到预抓取点 → 目标点 →
   读回末端位姿打印运动误差，再回到预抓取点。
3. 按 ``q`` / ``Esc`` 退出。

注意：``--execute`` 时机械臂会真实运动，运行前清空周边空间、保持急停可触及，
并由操作者全程观察。
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

SCRIPT_DIR = Path(__file__).resolve().parent          # Acc_test
EYE_IN_HAND_DIR = SCRIPT_DIR.parent                    # EyeInHandCalibrate
SOFTWARE_DIR = EYE_IN_HAND_DIR.parent                  # 3.Software
GRASPNET_DIR = SOFTWARE_DIR / "DummyGraspnet"
DEFAULT_HAND_EYE_DIR = EYE_IN_HAND_DIR / "hand_in" / "measured_data_in_dummyrobot"

# 复用 DummyGraspnet 中的单位安全换算与串口适配器（仅依赖 numpy / pyserial）。
sys.path.insert(0, str(GRASPNET_DIR))
from dummyrobot_adapter import (  # noqa: E402
    DummyRobotVCP,
    find_controller_port,
    load_handeye_transform,
    pose_mm_deg_to_transform_m,
    require_positive_z_mm,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--handeye", type=Path, default=DEFAULT_HAND_EYE_DIR,
                        help="手眼标定输出目录（含 rotation_matrix.txt / translation_vector.txt / mtx.txt / dist.txt）")
    parser.add_argument("--port", help="主控 USB 虚拟串口，例如 /dev/ttyACM0；默认按 VID 0x1209 自动查找")
    parser.add_argument("--timeout", type=float, default=1.5, help="读取主控位姿的超时秒数")
    parser.add_argument("--width", type=int, default=1280, help="相机彩色分辨率宽（须与标定一致，默认 1280）")
    parser.add_argument("--height", type=int, default=720, help="相机彩色分辨率高（须与标定一致，默认 720）")
    parser.add_argument("--fps", type=int, default=30, help="相机帧率")
    parser.add_argument("--min-depth-m", type=float, default=0.05, help="有效最小深度，米")
    parser.add_argument("--approach-mm", type=float, default=50.0,
                        help="预抓取点相对目标点的基座系 Z 向抬升高度，毫米")
    parser.add_argument("--speed-deg-s", type=float, default=30.0, help="MoveL 命令附带的速度参数")
    parser.add_argument("--motion-timeout-s", type=float, default=45.0, help="等待到位超时秒数")
    parser.add_argument("--tolerance-mm", type=float, default=3.0, help="判断到达目标的 XYZ 容差，毫米")
    parser.add_argument("--hold-seconds", type=float, default=2.0, help="到达目标点后停留时间，秒")
    parser.add_argument("--execute", action="store_true",
                        help="允许发送真实运动指令；不加此参数只做干运行")
    return parser.parse_args()


class AlignedRealsense:
    """D435 彩色/深度对齐采集，深度对齐到彩色流。"""

    def __init__(self, width=1280, height=720, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

    def get_frames(self):
        """返回 (color_bgr, depth_raw)；深度已对齐到彩色像素。"""
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color = np.asanyarray(aligned.get_color_frame().get_data())
        depth = np.asanyarray(aligned.get_depth_frame().get_data())
        return color, depth

    def stop(self):
        self.pipeline.stop()


def load_intrinsics(directory):
    """加载标定输出的相机内参与畸变系数。"""
    mtx = np.loadtxt(directory / "mtx.txt", dtype=np.float64).reshape(3, 3)
    dist = np.loadtxt(directory / "dist.txt", dtype=np.float64).reshape(1, -1)
    return mtx, dist


def pixel_to_camera_m(x, y, depth_m, mtx, dist):
    """像素 + 深度 -> 相机系三维点（米），带去畸变。"""
    normalized = cv2.undistortPoints(
        np.array([[[x, y]]], dtype=np.float64), mtx, dist, P=None,
    )
    nx, ny = normalized[0][0]
    return np.array([nx * depth_m, ny * depth_m, depth_m])


class HandEyeAccTest:
    def __init__(self, args):
        self.args = args
        self.robot = None
        self.camera = None
        self.mtx, self.dist = load_intrinsics(args.handeye)
        self.handeye = load_handeye_transform(str(args.handeye))

        self.camera = AlignedRealsense(args.width, args.height, args.fps)
        print(f"相机已启动：{args.width}x{args.height}@{args.fps}，"
              f"depth_scale={self.camera.depth_scale:g} m/unit")

        try:
            port = args.port or find_controller_port()
            self.robot = DummyRobotVCP(port, timeout=args.timeout)
            print(f"已连接主控 USB 虚拟串口：{port}（只读查询 #GETLPOS）")
            pose = self.robot.get_current_pose()
            require_positive_z_mm(pose, "当前末端位姿")
            print(f"当前末端位姿 [mm, deg]: {np.round(pose, 2).tolist()}")
        except Exception:
            self.camera.stop()
            self.camera = None
            raise

        self.enabled = False          # 是否已发送过 !START
        self.measured_points = []     # 记录每次点击估算的基座系坐标（米），用于一致性检查
        self.last_color = None
        self.last_depth = None
        self._pose_cache = None       # 低频刷新的 (T_base_end)，供标记绘制，避免逐帧查询串口
        self._pose_cache_time = 0.0

    # ------------------------------------------------------------------ #
    # 坐标换算
    # ------------------------------------------------------------------ #
    def current_pose(self):
        pose = self.robot.get_current_pose()
        require_positive_z_mm(pose, "当前末端位姿")
        return np.asarray(pose, dtype=float)

    def pixel_to_base_m(self, x, y):
        depth_raw = self.last_depth[y, x]
        depth_m = float(depth_raw) * self.camera.depth_scale
        if not np.isfinite(depth_m) or depth_m <= self.args.min_depth_m:
            raise ValueError(
                f"像素 ({x}, {y}) 处深度无效（raw={depth_raw}, {depth_m*1000:.1f} mm），"
                "请换一个深度有效且不过近的点。"
            )
        point_camera = pixel_to_camera_m(x, y, depth_m, self.mtx, self.dist)
        pose = self.current_pose()
        t_base_end = pose_mm_deg_to_transform_m(pose)
        point_end = self.handeye @ np.r_[point_camera, 1.0]
        point_base = t_base_end @ point_end
        return point_base[:3], point_camera, pose

    # ------------------------------------------------------------------ #
    # 运动执行
    # ------------------------------------------------------------------ #
    def wait_for_pose(self, target_mm, label):
        target = np.asarray(target_mm[:3], dtype=float)
        deadline = time.monotonic() + self.args.motion_timeout_s
        last = None
        while time.monotonic() < deadline:
            try:
                last = np.asarray(self.robot.get_current_pose()[:3], dtype=float)
            except TimeoutError:
                time.sleep(0.2)
                continue
            if np.linalg.norm(last - target) <= self.args.tolerance_mm:
                return last
            time.sleep(0.2)
        raise TimeoutError(
            f"{label} 未在 {self.args.motion_timeout_s:g}s 内到位："
            f"target={np.round(target, 1).tolist()} mm, "
            f"last={None if last is None else np.round(last, 1).tolist()} mm"
        )

    def move_l(self, pose_mm_deg, label):
        require_positive_z_mm(pose_mm_deg, label)
        reply = self.robot.move_l(np.asarray(pose_mm_deg, dtype=float),
                                  confirmed=True, speed_deg_s=self.args.speed_deg_s)
        print(f"  {label} 主控回执: {reply}")
        actual = self.wait_for_pose(pose_mm_deg, label)
        return actual

    # ------------------------------------------------------------------ #
    # 点击回调
    # ------------------------------------------------------------------ #
    def on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.last_color is None:
            return
        try:
            point_base, point_camera, pose = self.pixel_to_base_m(x, y)
        except (ValueError, TimeoutError, OSError) as exc:
            print(f"[跳过] 点击 ({x}, {y}) 处理失败: {exc}")
            return

        abc_deg = pose[3:]
        target_mm = np.r_[point_base * 1000.0, abc_deg]
        approach_mm = target_mm.copy()
        approach_mm[2] += self.args.approach_mm

        self.measured_points.append(point_base.copy())
        print("\n" + "=" * 60)
        print(f"点击像素 ({x}, {y})  深度 {point_camera[2]*1000:.1f} mm")
        print(f"相机系点 [m]: {np.round(point_camera, 4).tolist()}")
        print(f"基座系点 [m]: {np.round(point_base, 4).tolist()}")
        print(f"目标 MoveL [mm, deg]: {np.round(target_mm, 2).tolist()}")
        print(f"预抓取 MoveL [mm, deg]: {np.round(approach_mm, 2).tolist()}")

        if not self.args.execute:
            print("干运行：未发送运动指令。加 --execute 才会真实移动。")
            return

        if input('输入 "MOVE" 确认移动到此点（其他任意键取消）: ').strip() != "MOVE":
            print("已取消本次移动。")
            return

        try:
            if not self.enabled:
                print("使能机械臂回执:", self.robot.set_enabled(True))
                self.enabled = True

            self.move_l(approach_mm, "移动到预抓取点")
            self.move_l(target_mm, "移动到目标点")

            actual = np.asarray(self.robot.get_current_pose()[:3], dtype=float)
            motion_error_mm = float(np.linalg.norm(actual - target_mm[:3]))
            print(f"到达目标后读回 [mm]: {np.round(actual, 2).tolist()}")
            print(f"运动误差（读回 vs 目标）: {motion_error_mm:.2f} mm")
            print("提示：此误差反映机械臂定位精度；手眼是否准确请目视检查"
                  "末端/工具是否落在你点击的物理点上。")

            if self.args.hold_seconds > 0:
                time.sleep(self.args.hold_seconds)
            self.move_l(approach_mm, "返回预抓取点")
        except (TimeoutError, OSError, ValueError, PermissionError) as exc:
            print(f"[运动失败] {exc}")
            try:
                print("发送 !STOP 回执:", self.robot.stop())
            except Exception:
                pass
        print("=" * 60)

    # ------------------------------------------------------------------ #
    # 主循环
    # ------------------------------------------------------------------ #
    def run(self):
        window = "HandEye Acc Test (DummyRobot)"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window, self.on_click)

        print("\n左键点击目标点；q/Esc 退出；p 打印已记录点的一致性误差；c 清空记录。")
        if not self.args.execute:
            print("当前为干运行模式，点击只计算不移动。")

        try:
            while True:
                self.last_color, self.last_depth = self.camera.get_frames()
                show = self.last_color.copy()
                self._refresh_pose_cache()
                for point_base in self.measured_points:
                    # 把已记录的基座点投影回当前图像（用于观察，非精度依据）。
                    self._draw_marker(show, point_base)
                cv2.imshow(window, show)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
                if key == ord("c"):
                    self.measured_points.clear()
                    print("已清空记录点。")
                if key == ord("p"):
                    self._print_consistency()
        except KeyboardInterrupt:
            print("\n用户中断。")
        finally:
            self.cleanup()

    def _refresh_pose_cache(self):
        """以约 2 Hz 刷新一次末端位姿缓存，供标记绘制；失败时保留上次结果。"""
        now = time.monotonic()
        if now - self._pose_cache_time < 0.5:
            return
        try:
            pose = np.asarray(self.robot.get_current_pose(), dtype=float)
            require_positive_z_mm(pose, "当前末端位姿")
            self._pose_cache = pose_mm_deg_to_transform_m(pose)
            self._pose_cache_time = now
        except (TimeoutError, OSError, ValueError):
            pass

    def _draw_marker(self, image, point_base):
        """把已记录的基座点粗略投影回图像（用于观察，非精度依据）。"""
        t_base_end = self._pose_cache
        if t_base_end is None:
            return
        t_end_base = np.linalg.inv(t_base_end)
        t_end_camera_inv = np.linalg.inv(self.handeye)
        p_end = t_end_base @ np.r_[point_base, 1.0]
        p_cam = t_end_camera_inv @ p_end
        z = p_cam[2]
        if z <= 1e-3:
            return
        u = self.mtx[0, 0] * p_cam[0] / z + self.mtx[0, 2]
        v = self.mtx[1, 1] * p_cam[1] / z + self.mtx[1, 2]
        cv2.circle(image, (int(round(u)), int(round(v))), 4, (0, 255, 0), -1)

    def _print_consistency(self):
        points = np.asarray(self.measured_points, dtype=float)
        if len(points) < 2:
            print(f"当前只有 {len(points)} 个记录点，至少需要 2 个（从不同位姿点击同一个物理点）。")
            return
        centroid = points.mean(axis=0)
        deviations = np.linalg.norm(points - centroid, axis=1)
        span = float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))
        print("\n----- 一致性检查（点击同一物理点所得基座坐标的离散程度）-----")
        for index, (point, dev) in enumerate(zip(points, deviations), 1):
            print(f"  点 {index}: {np.round(point, 4).tolist()} m, 距均值 {dev*1000:.1f} mm")
        print(f"  样本数 {len(points)}；包围盒对角线 {span*1000:.1f} mm；"
              f"最大偏差 {deviations.max()*1000:.1f} mm；RMS {np.sqrt((deviations**2).mean())*1000:.1f} mm")
        print("  说明：该值越小说明手眼外参越准；它综合了手眼误差、深度噪声与点选误差。")
        print("-" * 60)

    def cleanup(self):
        try:
            if getattr(self, "robot", None) is not None:
                self.robot.close()
        finally:
            if getattr(self, "camera", None) is not None:
                self.camera.stop()
            cv2.destroyAllWindows()


def main():
    args = parse_args()
    if not args.handeye.is_dir():
        raise FileNotFoundError(f"手眼标定目录不存在: {args.handeye}")
    for name in ("rotation_matrix.txt", "translation_vector.txt", "mtx.txt", "dist.txt"):
        if not (args.handeye / name).is_file():
            raise FileNotFoundError(f"缺少手眼标定结果文件: {args.handeye / name}")
    if args.approach_mm <= 0 or args.tolerance_mm <= 0 or args.speed_deg_s <= 0:
        raise ValueError("approach-mm / tolerance-mm / speed-deg-s 必须为正")
    if not args.execute:
        print("安全默认：未发送任何机械臂指令。确认现场防护后加 --execute 才会进入实机流程。")

    tester = HandEyeAccTest(args)
    tester.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断。")
        sys.exit(130)
    except (OSError, RuntimeError, TimeoutError, ValueError, FileNotFoundError) as exc:
        print(f"\n错误: {exc}")
        sys.exit(1)
