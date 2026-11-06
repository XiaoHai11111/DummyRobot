# coding=utf-8
"""Collect RealSense images and DummyRobot end-effector poses for eye-in-hand calibration.

The controller's USB virtual COM port uses its ASCII protocol. ``#GETLPOS``
returns ``okl X Y Z A B C`` where XYZ are millimeters and ABC are degrees.
Calibration pose files use meters and radians, matching OpenCV and the UR5
collector that this script replaces.
"""

import argparse
import os
import re
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import serial
from serial.tools import list_ports

from calibration_settings import (
    CALIBRATION_DATA_DIR,
    CHESSBOARD_COLS,
    CHESSBOARD_ROWS,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_SAVE_PATH = os.path.join(SCRIPT_DIR, CALIBRATION_DATA_DIR)
CHESSBOARD_SIZE = (CHESSBOARD_COLS, CHESSBOARD_ROWS)
DEFAULT_BAUDRATE = 115200  # USB CDC ignores baud rate, but pyserial requires a value.
POSE_RESPONSE = re.compile(
    r"^okl\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)"
    r"\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)$"
)


def find_controller_port():
    """Find the controller's virtual COM port by its configured USB VID."""
    ports = list(list_ports.comports())
    for port in ports:
        if port.vid == 0x1209:
            return port.device

    available = ", ".join(port.device for port in ports) or "未发现串口"
    raise RuntimeError(
        "没有找到 USB VID 为 0x1209 的主控虚拟串口。"
        f"可用串口: {available}。请使用 --port 指定串口。"
    )


class DummyRobotVCP:
    """Small client for the controller's USB CDC ASCII command interface."""

    def __init__(self, port, baudrate=DEFAULT_BAUDRATE, timeout=1.0):
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=0.1, write_timeout=timeout)
        self.timeout = timeout
        # Allow the USB CDC device to finish enumerating after the port opens.
        time.sleep(0.2)
        self.serial.reset_input_buffer()

    def get_current_pose(self):
        """Return [x, y, z, rx, ry, rz] in meters and radians."""
        self.serial.reset_input_buffer()
        self.serial.write(b"#GETLPOS\r\n")
        self.serial.flush()

        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            line = self.serial.readline().decode("ascii", errors="replace").strip()
            match = POSE_RESPONSE.match(line)
            if match:
                values = [float(value) for value in match.groups()]
                xyz_m = [value / 1000.0 for value in values[:3]]
                rpy_rad = np.deg2rad(values[3:]).tolist()
                return xyz_m + rpy_rad

        raise TimeoutError(
            f"主控未在 {self.timeout:g} 秒内返回末端位姿。"
            "请确认 USB 虚拟串口、固件 ASCII 协议和主控连接正常。"
        )

    def close(self):
        if self.serial.is_open:
            self.serial.close()


def next_capture_index(image_dir):
    """Continue after the highest existing numeric image index."""
    indices = []
    for name in os.listdir(image_dir):
        stem, extension = os.path.splitext(name)
        if extension.lower() != ".jpg":
            continue
        if stem.startswith("images"):
            stem = stem[len("images"):]
        if stem.isdigit():
            indices.append(int(stem))
    return max(indices, default=-1) + 1


def data_collect(robot, image_save_path=IMAGE_SAVE_PATH):
    os.makedirs(image_save_path, exist_ok=True)
    count = next_capture_index(image_save_path)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    started = False
    try:
        pipeline.start(config)
        started = True
        print(
            f"标定板内角点规格：{CHESSBOARD_COLS}x{CHESSBOARD_ROWS}，"
            "按 s 采集，按 q 或 Esc 退出。"
        )
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            board_found, corners = cv2.findChessboardCorners(
                gray,
                CHESSBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            preview = color_image.copy()
            if board_found:
                cv2.drawChessboardCorners(preview, CHESSBOARD_SIZE, corners, board_found)
                status = "Board detected - press s to capture"
                status_color = (0, 200, 0)
            else:
                status = "Board not detected - adjust view/lighting"
                status_color = (0, 0, 255)
            cv2.putText(
                preview,
                status,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                status_color,
                2,
                cv2.LINE_AA,
            )
            cv2.namedWindow(
                "detection",
                flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
            )
            cv2.imshow("detection", preview)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                if not board_found:
                    print("当前画面未识别到完整的 7x10 内角点，本次不采集。")
                    continue
                print(f"采集第 {count} 组数据…")
                try:
                    pose = robot.get_current_pose()
                except (TimeoutError, serial.SerialException, ValueError) as exc:
                    print(f"获取机械臂位姿失败，本次未保存：{exc}")
                    continue

                image_path = os.path.join(image_save_path, f"{count}.jpg")
                if not cv2.imwrite(image_path, color_image):
                    print(f"图像保存失败：{image_path}")
                    continue

                pose_path = os.path.join(image_save_path, "poses.txt")
                try:
                    with open(pose_path, "a", encoding="utf-8") as pose_file:
                        pose_file.write(",".join(f"{value:.9f}" for value in pose) + "\n")
                except OSError:
                    os.remove(image_path)
                    raise

                print(f"末端位姿 [m, rad]: {pose}")
                print(f"图像和位姿已保存：{image_path}")
                count += 1
            elif key == ord("q") or key == 27:
                break
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="通过 USB 虚拟串口采集 DummyRobot 眼在手上标定数据")
    parser.add_argument(
        "--port",
        default=os.environ.get("DUMMYROBOT_PORT"),
        help="主控 USB 虚拟串口，例如 Linux /dev/ttyACM0、Windows COM5；默认按 USB VID 自动查找",
    )
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="串口波特率（USB CDC 使用占位值）")
    parser.add_argument("--timeout", type=float, default=1.0, help="读取机械臂位姿的超时秒数")
    args = parser.parse_args()

    robot = None
    try:
        port = args.port or find_controller_port()
        robot = DummyRobotVCP(port, baudrate=args.baudrate, timeout=args.timeout)
        print(f"已连接主控 USB 虚拟串口：{port}")
        # Fail early when the port is wrong or the controller does not answer.
        pose = robot.get_current_pose()
        print(f"主控位姿读取正常 [m, rad]: {pose}")
        data_collect(robot)
    except (OSError, serial.SerialException, RuntimeError, TimeoutError) as exc:
        parser.error(str(exc))
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
