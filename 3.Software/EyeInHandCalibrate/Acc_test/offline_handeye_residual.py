#!/usr/bin/env python3
# coding=utf-8
"""离线手眼标定残差验证（不接实机、不接相机）。

只使用已经采集好的棋盘图片 + 末端位姿文件 + 已保存的标定结果，计算三个量化指标：

1. 相机 2D 重投影误差（像素）
   用保存的内参/畸变对每张图重新 solvePnP，把棋盘角点投影回图像，与检测到的
   角点比较。用于发现糊图、反光、角点识别错误的坏样本。

2. 手眼 AX=XB 残差（旋转 ° / 平移 mm）
   对每一对位姿 (i, j) 构造相对运动 A_ij、B_ij，计算
   E = (A_ij · X) · (X · B_ij)^(-1)，X 为手眼外参 T_end_camera。
   E 越接近单位矩阵，说明手眼外参越满足 AX=XB 约束。这是手眼外参本身的量化残差。

3. 棋盘基座系一致性（位置 mm / 朝向 °）
   棋盘固定不动，因此任意位姿下
   T_base_board_i = T_base_end_i · X · T_cam_board_i
   都应当相同。统计各次估计的棋盘原点位置离散度和法向朝向离散度。

运行（DummyRobot 环境即可，无需任何硬件）：
    conda activate DummyRobot
    cd 3.Software/EyeInHandCalibrate/Acc_test
    python offline_handeye_residual.py
可选参数 --data-dir、--result-dir、--board-cols/--board-rows/--square-m。
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent                # Acc_test
EYE_IN_HAND_DIR = SCRIPT_DIR.parent                         # EyeInHandCalibrate
HAND_IN_DIR = EYE_IN_HAND_DIR / "hand_in"
DEFAULT_DATA_DIR = HAND_IN_DIR / "collect_data_in_dummyrobot"
DEFAULT_RESULT_DIR = HAND_IN_DIR / "measured_data_in_dummyrobot"

# 复用标定配置（7x10 内角点、0.015 m）。
sys.path.insert(0, str(HAND_IN_DIR))
from calibration_settings import (  # noqa: E402
    CHESSBOARD_COLS,
    CHESSBOARD_ROWS,
    CHECKER_SQUARE_SIZE_M,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR,
                        help="采集数据目录（含 *.jpg 与 poses.txt）")
    parser.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR,
                        help="标定结果目录（含 mtx/dist/rotation/translation）")
    parser.add_argument("--board-cols", type=int, default=CHESSBOARD_COLS, help="内角点列数")
    parser.add_argument("--board-rows", type=int, default=CHESSBOARD_ROWS, help="内角点行数")
    parser.add_argument("--square-m", type=float, default=CHECKER_SQUARE_SIZE_M, help="方格边长，米")
    return parser.parse_args()


# --------------------------------------------------------------------------- #
# 位姿与矩阵工具（与 hand_in_eye_calibrate.py 保持同一约定）
# --------------------------------------------------------------------------- #
def euler_to_rotation(rx, ry, rz):
    """R = Rz @ Ry @ Rx，与标定脚本一致。"""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_homogeneous(R, t):
    t = np.asarray(t, dtype=float).reshape(3)
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = t
    return T


def rotation_angle_deg(R):
    """旋转矩阵 -> 旋转角（度），用 Rodrigues 更稳。"""
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=float))
    return float(np.rad2deg(np.linalg.norm(rvec)))


def load_poses(path):
    """读取 poses.txt：每行 6 个数 [x,y,z,rx,ry,rz]，单位米/弧度。"""
    poses = []
    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                values = [float(v) for v in line.split(",")]
            except ValueError:
                print(f"警告: poses.txt 第 {line_no} 行无法解析，跳过: {line!r}")
                continue
            if len(values) != 6:
                print(f"警告: poses.txt 第 {line_no} 行不是 6 个数，跳过")
                continue
            x, y, z, rx, ry, rz = values
            poses.append(make_homogeneous(euler_to_rotation(rx, ry, rz), [x, y, z]))
    return poses


def load_intrinsics(directory):
    mtx = np.loadtxt(directory / "mtx.txt", dtype=np.float64).reshape(3, 3)
    dist = np.loadtxt(directory / "dist.txt", dtype=np.float64).reshape(1, -1)
    return mtx, dist


def load_handeye(directory):
    """T_end_camera = [rotation_matrix | translation_vector]，平移单位米。"""
    R = np.loadtxt(directory / "rotation_matrix.txt", dtype=np.float64).reshape(3, 3)
    t = np.loadtxt(directory / "translation_vector.txt", dtype=np.float64).reshape(3)
    return make_homogeneous(R, t)


def sorted_image_files(data_dir):
    files = [p for p in data_dir.glob("*.jpg")]
    files.sort(key=lambda p: int(p.stem[len("images"):] if p.stem.startswith("images") else p.stem))
    return files


def build_board_points(cols, rows, square_m):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return objp


# --------------------------------------------------------------------------- #
# 每张图检测角点 + 求解棋盘在相机系位姿
# --------------------------------------------------------------------------- #
def detect_board_pose(image_path, objp, mtx, dist, cols, rows):
    img = cv2.imread(str(image_path))
    if img is None:
        raise ValueError(f"无法读取图片 {image_path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    if not ret:
        return None, None, None
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    _, rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    R, _ = cv2.Rodrigues(rvec)
    T_cam_board = make_homogeneous(R, tvec)
    return T_cam_board, corners2, objp


# --------------------------------------------------------------------------- #
# 指标 1：相机 2D 重投影误差
# --------------------------------------------------------------------------- #
def camera_reprojection_errors(T_cam_board_list, corners_list, objp_list, mtx, dist):
    per_image = []
    all_errors = []
    for T_cam_board, corners, objp in zip(T_cam_board_list, corners_list, objp_list):
        rvec, _ = cv2.Rodrigues(T_cam_board[:3, :3])
        projected, _ = cv2.projectPoints(objp, rvec, T_cam_board[:3, 3], mtx, dist)
        projected = projected.reshape(-1, 2)
        err = np.linalg.norm(projected - corners.reshape(-1, 2), axis=1)
        per_image.append((float(err.mean()), float(err.max())))
        all_errors.append(err)
    all_errors = np.concatenate(all_errors) if all_errors else np.array([np.nan])
    return per_image, all_errors


# --------------------------------------------------------------------------- #
# 指标 2：手眼 AX=XB 残差
# --------------------------------------------------------------------------- #
def handeye_pairwise_residuals(T_base_end_list, T_cam_board_list, X):
    rot_errs = []
    trans_errs = []
    n = len(T_base_end_list)
    for i in range(n):
        Ti, Bi = T_base_end_list[i], T_cam_board_list[i]
        for j in range(i + 1, n):
            Tj, Bj = T_base_end_list[j], T_cam_board_list[j]
            A = np.linalg.inv(Ti) @ Tj          # 末端相对运动 i -> j
            B = Bi @ np.linalg.inv(Bj)          # 相机相对运动 i -> j（棋盘固定）
            E = A @ X @ np.linalg.inv(X @ B)    # 理想情况下应为单位矩阵
            rot_errs.append(rotation_angle_deg(E[:3, :3]))
            trans_errs.append(float(np.linalg.norm(E[:3, 3]) * 1000.0))
    return np.asarray(rot_errs), np.asarray(trans_errs)


# --------------------------------------------------------------------------- #
# 指标 3：棋盘基座系一致性
# --------------------------------------------------------------------------- #
def board_base_consistency(T_base_end_list, T_cam_board_list, X):
    positions = []
    normals = []
    for T_end, T_cam_board in zip(T_base_end_list, T_cam_board_list):
        T_base_board = T_end @ X @ T_cam_board
        positions.append(T_base_board[:3, 3])
        normals.append(T_base_board[:3, 2])   # 棋盘法向（Z 轴）在基座系的方向
    positions = np.asarray(positions)
    normals = np.asarray(normals)

    centroid = positions.mean(axis=0)
    pos_dev = np.linalg.norm(positions - centroid, axis=1) * 1000.0

    mean_normal = normals.mean(axis=0)
    mean_normal = mean_normal / (np.linalg.norm(mean_normal) + 1e-12)
    cosang = np.clip(normals @ mean_normal, -1.0, 1.0)
    ang_dev = np.rad2deg(np.arccos(cosang))
    return positions, normals, pos_dev, ang_dev


def stats(arr, unit):
    arr = np.asarray(arr, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return "  (无数据)"
    return (f"mean={arr.mean():.4f} {unit}   median={np.median(arr):.4f} {unit}   "
            f"max={arr.max():.4f} {unit}   p90={np.percentile(arr, 90):.4f} {unit}")


def main():
    args = parse_args()
    if not args.data_dir.is_dir():
        raise FileNotFoundError(f"采集数据目录不存在: {args.data_dir}")
    if not args.result_dir.is_dir():
        raise FileNotFoundError(f"标定结果目录不存在: {args.result_dir}")

    images = sorted_image_files(args.data_dir)
    poses_path = args.data_dir / "poses.txt"
    if not poses_path.is_file():
        raise FileNotFoundError(f"缺少位姿文件: {poses_path}")
    if not images:
        raise FileNotFoundError(f"{args.data_dir} 下没有 *.jpg 图片")

    mtx, dist = load_intrinsics(args.result_dir)
    X = load_handeye(args.result_dir)
    T_base_end_list = load_poses(poses_path)
    objp = build_board_points(args.board_cols, args.board_rows, args.square_m)

    print("=" * 70)
    print("离线手眼标定残差验证")
    print(f"采集数据目录: {args.data_dir}")
    print(f"标定结果目录: {args.result_dir}")
    print(f"棋盘内角点: {args.board_cols}x{args.board_rows}  方格边长: {args.square_m} m")
    print(f"图片数: {len(images)}   位姿数: {len(T_base_end_list)}")
    if len(images) != len(T_base_end_list):
        print("!! 警告: 图片数与位姿数不一致，将按顺序截断配对，请检查数据。")

    n = min(len(images), len(T_base_end_list))
    T_cam_board_list = []
    corners_list = []
    objp_list = []
    used_indices = []
    skipped = []
    for k in range(n):
        try:
            T_cam_board, corners, _objp = detect_board_pose(
                images[k], objp, mtx, dist, args.board_cols, args.board_rows)
        except Exception as exc:
            print(f"图片 {images[k].name} 处理失败: {exc}")
            skipped.append(k)
            continue
        if T_cam_board is None:
            skipped.append(k)
            print(f"图片 {images[k].name}: 未识别到 {args.board_cols}x{args.board_rows} 内角点，跳过。")
            continue
        T_cam_board_list.append(T_cam_board)
        corners_list.append(corners)
        objp_list.append(_objp)
        used_indices.append(k)

    if len(T_cam_board_list) < 2:
        raise RuntimeError("有效图片少于 2 张，无法计算残差。请检查采集数据。")

    T_base_used = [T_base_end_list[k] for k in used_indices]
    print(f"有效配对图片: {len(T_cam_board_list)} / {n}" + ("（含跳过）" if skipped else ""))
    print("=" * 70)

    # 指标 1
    per_image, all_err = camera_reprojection_errors(
        T_cam_board_list, corners_list, objp_list, mtx, dist)
    print("\n[1] 相机 2D 重投影误差 (像素)")
    for idx, (img_index, (mean_px, max_px)) in enumerate(zip(used_indices, per_image)):
        flag = "  <-- 偏大" if max_px > 1.5 else ""
        print(f"    {images[img_index].name:>8s}: mean={mean_px:.4f} px  max={max_px:.4f} px{flag}")
    print(f"    汇总: {stats(all_err, 'px')}")
    print("    判据建议: mean < 0.5 px；max > 1.5 px 的图片建议剔除后重新标定。")

    # 指标 2
    rot_errs, trans_errs = handeye_pairwise_residuals(T_base_used, T_cam_board_list, X)
    print("\n[2] 手眼 AX=XB 残差")
    print(f"    旋转误差: {stats(rot_errs, 'deg')}")
    print(f"    平移误差: {stats(trans_errs, 'mm')}")
    print("    判据建议: 旋转 mean < 0.5°、平移 mean < 3 mm 为可接受；明显偏大说明手眼外参或位姿数据有问题。")

    # 指标 3
    positions, normals, pos_dev, ang_dev = board_base_consistency(
        T_base_used, T_cam_board_list, X)
    print("\n[3] 棋盘基座系一致性（棋盘固定，理应每次相同）")
    print(f"    棋盘原点位置离散度: {stats(pos_dev, 'mm')}")
    print(f"    棋盘法向朝向离散度: {stats(ang_dev, 'deg')}")
    print("    判据建议: 位置 RMS < 3 mm、朝向 < 1°；该指标同时含手眼误差与角点/位姿噪声。")

    print("\n" + "=" * 70)
    print("说明: 指标 1 反映相机内参与采集质量；指标 2、3 反映手眼外参 X 本身的一致性。")
    print("若指标 1 良好而指标 2/3 偏大，优先怀疑：位姿多样性不足、机械臂零点错误、")
    print("采集时棋盘/相机有移动、或 poses.txt 单位/顺序有误。")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, FileNotFoundError, RuntimeError) as exc:
        print(f"\n错误: {exc}")
        sys.exit(1)
