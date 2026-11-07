#!/usr/bin/env python3
"""Local HTTP bridge for DummyWeb: Linux serial, RealSense RGB-D, and robot models."""

import json
import mimetypes
import os
import re
import threading
import time
import urllib.parse
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import serial
from serial.tools import list_ports


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
CONFIG_PATH = BASE_DIR / "config.json"
CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

JOINT_RESPONSE = re.compile(r"^okj\s+([-+\d.eE]+(?:\s+[-+\d.eE]+){5})$")
POSE_RESPONSE = re.compile(r"^okl\s+([-+\d.eE]+(?:\s+[-+\d.eE]+){5})$")
FLOAT_TOKEN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def numbers(value, count, fallback):
    vals = [float(v) for v in (value or fallback).split()]
    return (vals + [0.0] * count)[:count]


def xyz(value):
    vals = [float(v) for v in value.split()] if value else []
    return (vals + [0.0, 0.0, 0.0])[:3]


def parse_color(element, material_map=None):
    if element is None:
        return [0.78, 0.84, 0.9, 1.0]
    rgba = element.get("rgba")
    if rgba:
        return numbers(rgba, 4, "0.8 0.84 0.9 1")
    material = element.get("material")
    if material and material_map and material in material_map:
        return material_map[material]
    return [0.78, 0.84, 0.9, 1.0]


def parse_urdf(path):
    root_xml = ET.parse(path).getroot()
    mesh_dir = Path(CONFIG["models"]["mesh_dir"]).resolve()
    links = {}
    for link_xml in root_xml.findall("link"):
        link_name = link_xml.get("name")
        link = {"name": link_name, "origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
                "joint": None, "visuals": [], "children": []}
        for visual in link_xml.findall("visual"):
            visual_origin = visual.find("origin")
            origin = {"xyz": xyz(visual_origin.get("xyz") if visual_origin is not None else None),
                      "rpy": xyz(visual_origin.get("rpy") if visual_origin is not None else None)}
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            mesh = geometry.find("mesh")
            if mesh is not None:
                filename = os.path.basename(mesh.get("filename", ""))
                scale = xyz(mesh.get("scale", "1 1 1"))
                if (mesh_dir / filename).is_file():
                    material = visual.find("material/color")
                    link["visuals"].append({"kind": "mesh", "file": filename,
                                            "scale": scale, "origin": origin,
                                            "color": numbers(material.get("rgba") if material is not None else None,
                                                             4, "0.78 0.84 0.9 1")})
            else:
                for shape in ("box", "cylinder", "sphere"):
                    shape_xml = geometry.find(shape)
                    if shape_xml is not None:
                        material = visual.find("material/color")
                        link["visuals"].append({"kind": shape, "dimensions": shape_xml.attrib,
                                                "origin": origin,
                                                "color": numbers(material.get("rgba") if material is not None else None,
                                                                 4, "0.78 0.84 0.9 1")})
        links[link_name] = link

    child_links = set()
    for joint_xml in root_xml.findall("joint"):
        parent_xml, child_xml = joint_xml.find("parent"), joint_xml.find("child")
        if parent_xml is None or child_xml is None:
            continue
        parent_name, child_name = parent_xml.get("link"), child_xml.get("link")
        if parent_name not in links or child_name not in links:
            continue
        child = links[child_name]
        origin = joint_xml.find("origin")
        axis = joint_xml.find("axis")
        limit = joint_xml.find("limit")
        child["origin"] = {"xyz": xyz(origin.get("xyz") if origin is not None else None),
                           "rpy": xyz(origin.get("rpy") if origin is not None else None)}
        child["joint"] = {"name": joint_xml.get("name"), "type": joint_xml.get("type", "fixed"),
                          "axis": xyz(axis.get("xyz") if axis is not None else "0 0 1"),
                          "lower": float(limit.get("lower", "0")) if limit is not None else 0,
                          "upper": float(limit.get("upper", "0")) if limit is not None else 0}
        links[parent_name]["children"].append(child)
        child_links.add(child_name)
    roots = [link for name, link in links.items() if name not in child_links]
    if len(roots) != 1:
        raise ValueError(f"URDF needs one root link; found {[r['name'] for r in roots]}")
    return {"format": "urdf", "name": root_xml.get("name", "Robot"), "root": roots[0]}


def parse_mjcf(path):
    root_xml = ET.parse(path).getroot()
    mesh_dir = Path(CONFIG["models"]["mesh_dir"]).resolve()
    mesh_files = {m.get("name"): os.path.basename(m.get("file", ""))
                  for m in root_xml.findall("./asset/mesh")}
    materials = {m.get("name"): numbers(m.get("rgba"), 4, "0.8 0.84 0.9 1")
                 for m in root_xml.findall("./asset/material")}

    def parse_body(body_xml):
        body = {"name": body_xml.get("name", "body"),
                "origin": {"xyz": xyz(body_xml.get("pos")),
                           "quat": numbers(body_xml.get("quat"), 4, "1 0 0 0")},
                "joint": None, "visuals": [], "children": []}
        joint_xml = body_xml.find("joint")
        if joint_xml is not None:
            limit = numbers(joint_xml.get("range"), 2, "0 0")
            body["joint"] = {"name": joint_xml.get("name", body["name"] + "_joint"),
                             "type": joint_xml.get("type", "hinge"),
                             "axis": xyz(joint_xml.get("axis", "0 0 1")),
                             "lower": limit[0], "upper": limit[1]}
        for geom in body_xml.findall("geom"):
            geom_class = geom.get("class", "")
            if geom_class != "visual" and geom.get("group") != "2":
                continue
            mesh_name = geom.get("mesh")
            if not mesh_name or mesh_name not in mesh_files:
                continue
            filename = mesh_files[mesh_name]
            if not (mesh_dir / filename).is_file():
                continue
            body["visuals"].append({"kind": "mesh", "file": filename,
                                    "scale": [1, 1, 1],
                                    "origin": {"xyz": xyz(geom.get("pos")),
                                               "quat": numbers(geom.get("quat"), 4, "1 0 0 0")},
                                    "color": parse_color(geom, materials)})
        for child_xml in body_xml.findall("body"):
            body["children"].append(parse_body(child_xml))
        return body

    body_xml = root_xml.find("./worldbody/body")
    if body_xml is None:
        raise ValueError("MJCF has no worldbody/body")
    return {"format": "mjcf", "name": root_xml.get("model", "Robot"), "root": parse_body(body_xml)}


class RobotLink:
    def __init__(self):
        self.lock = threading.RLock()
        self.port = None
        self.device = None
        self.enabled = False
        self.last_joints = [float(v) for v in CONFIG["robot"]["resting_pose_deg"]]
        self.last_pose = None
        self.gripper_deg = float(CONFIG["robot"]["initial_gripper_angle_deg"])
        self.busy = ""
        self.last_error = ""

    @property
    def connected(self):
        return bool(self.device and self.device.is_open)

    def scan(self):
        ports = list(list_ports.comports())
        ports.sort(key=lambda p: (
            0 if p.vid is not None or p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB", "/dev/serial/")) else 1,
            p.device,
        ))
        return [{"device": p.device, "description": p.description,
                 "vid": p.vid, "pid": p.pid, "serial_number": p.serial_number}
                for p in ports]

    def connect(self, options):
        with self.lock:
            if self.connected:
                self.device.close()
            bytesizes = {5: serial.FIVEBITS, 6: serial.SIXBITS,
                         7: serial.SEVENBITS, 8: serial.EIGHTBITS}
            stopbits = {1: serial.STOPBITS_ONE, 1.5: serial.STOPBITS_ONE_POINT_FIVE,
                        2: serial.STOPBITS_TWO}
            port = options.get("port") or CONFIG["serial"]["default_port"]
            baud = int(options.get("baudrate", CONFIG["serial"]["baudrate"]))
            bits = int(options.get("bytesize", CONFIG["serial"]["bytesize"]))
            parity = str(options.get("parity", CONFIG["serial"]["parity"])).upper()
            stop = float(options.get("stopbits", CONFIG["serial"]["stopbits"]))
            if bits not in bytesizes or parity not in "NEO" or stop not in stopbits:
                raise ValueError("Unsupported serial framing settings")
            self.device = serial.Serial(port=port, baudrate=baud,
                                        bytesize=bytesizes[bits], parity=parity,
                                        stopbits=stopbits[stop],
                                        timeout=float(CONFIG["serial"]["read_timeout_s"]),
                                        write_timeout=1.0)
            self.port = port
            self.enabled = False  # state is local; firmware has no ASCII enabled-state query
            self.last_error = ""
            time.sleep(0.15)
            self.device.reset_input_buffer()
            return {"connected": True, "port": self.port}

    def disconnect(self):
        with self.lock:
            if self.device and self.device.is_open:
                self.device.close()
            self.device = None
            self.enabled = False
            self.port = None
            return {"connected": False}

    def _require(self):
        if not self.connected:
            raise RuntimeError("先连接机械臂串口")

    def command(self, command, matcher=None, timeout=None):
        self._require()
        with self.lock:
            self.device.reset_input_buffer()
            self.device.write((command + "\r\n").encode("ascii"))
            self.device.flush()
            deadline = time.monotonic() + (timeout or float(CONFIG["serial"]["command_timeout_s"]))
            lines = []
            while time.monotonic() < deadline:
                raw = self.device.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                lines.append(line)
                if matcher is None or matcher(line):
                    return line
            raise TimeoutError(f"串口命令超时: {command}; 收到: {lines}")

    def read_joints(self):
        line = self.command("#GETJPOS", lambda s: bool(JOINT_RESPONSE.match(s)))
        values = [float(v) for v in JOINT_RESPONSE.match(line).group(1).split()]
        self.last_joints = values
        return values

    def read_pose(self):
        line = self.command("#GETLPOS", lambda s: bool(POSE_RESPONSE.match(s)))
        self.last_pose = [float(v) for v in POSE_RESPONSE.match(line).group(1).split()]
        return self.last_pose

    def status(self):
        with self.lock:
            state = {"connected": self.connected, "port": self.port,
                     "enabled": self.enabled, "busy": self.busy,
                     "joints_deg": self.last_joints,
                     "gripper_deg": self.gripper_deg,
                     "pose_mm_deg": self.last_pose,
                     "error": self.last_error}
            if self.connected:
                try:
                    state["joints_deg"] = self.read_joints()
                    state["pose_mm_deg"] = self.read_pose()
                    self.last_error = ""
                    state["error"] = ""
                except Exception as exc:
                    self.last_error = str(exc)
                    state["error"] = self.last_error
            return state

    def set_enabled(self, enabled):
        self._require()
        expected = "Started ok" if enabled else "Disabled ok"
        line = self.command("!START" if enabled else "!DISABLE",
                            lambda s: expected in s or "ok" in s.lower())
        self.enabled = bool(enabled)
        return {"enabled": self.enabled, "reply": line}

    def move_joints(self, angles, internal=False):
        self._require()
        if self.busy and not internal:
            raise RuntimeError("Homing/Resting 正在执行，当前不能更改关节目标")
        if not self.enabled:
            raise RuntimeError("请先使能机械臂")
        limits = CONFIG["robot"]["joints"]
        angles = [float(v) for v in angles]
        if len(angles) != 6:
            raise ValueError("需要 6 个关节角")
        for value, limit in zip(angles, limits):
            if not limit["min_deg"] <= value <= limit["max_deg"]:
                raise ValueError(f"{limit['name']} 超出范围 [{limit['min_deg']}, {limit['max_deg']}]°")
        speed = float(CONFIG["robot"]["joint_speed_deg_s"])
        command = "&" + ",".join(f"{v:.2f}" for v in angles + [speed])
        line = self.command(command, lambda s: "ok" in s.lower(), timeout=float(CONFIG["serial"]["operation_timeout_s"]))
        self.last_joints = angles
        return {"reply": line, "joints_deg": angles}

    def set_gripper(self, angle):
        self._require()
        if self.busy:
            raise RuntimeError("Homing/Resting 正在执行，当前不能控制夹爪")
        if not self.enabled:
            raise RuntimeError("请先使能机械臂")
        limits = CONFIG["robot"]["gripper"]
        angle = float(angle)
        if not limits["min_deg"] <= angle <= limits["max_deg"]:
            raise ValueError(f"夹爪角度需在 [{limits['min_deg']}, {limits['max_deg']}]°")
        line = self.command(f"#GRIP {angle:.2f}", lambda s: "Grip angle" in s,
                            timeout=float(CONFIG["serial"]["operation_timeout_s"]))
        self.gripper_deg = angle
        return {"reply": line, "gripper_deg": angle}

    def operation(self, name):
        self._require()
        if not self.enabled:
            raise RuntimeError("请先使能机械臂")
        poses = {"homing": CONFIG["robot"]["homing_pose_deg"],
                 "resting": CONFIG["robot"]["resting_pose_deg"]}
        if name not in poses:
            raise ValueError("未知回零操作")
        self.busy = name
        try:
            self.move_joints(poses[name], internal=True)
            target = [float(v) for v in poses[name]]
            deadline = time.monotonic() + float(CONFIG["serial"]["operation_timeout_s"])
            tolerance = float(CONFIG["robot"]["joint_tolerance_deg"])
            while time.monotonic() < deadline:
                self.last_joints = self.read_joints()
                if all(abs(a - b) <= tolerance for a, b in zip(self.last_joints, target)):
                    return {"reply": "pose reached", "joints_deg": self.last_joints}
                time.sleep(0.2)
            raise TimeoutError(f"{name} 目标未在超时时间内到达; current={self.last_joints}")
        finally:
            self.busy = ""

    def stop(self):
        self._require()
        line = self.command("!STOP", lambda s: "Stopped ok" in s)
        return {"reply": line}

    def send_user_command(self, text):
        """Send one explicitly supported USB-CDC ASCII command from the UI."""
        self._require()
        command = str(text or "").strip()
        if not command or len(command) > 128 or not command.isascii() or "\r" in command or "\n" in command:
            raise ValueError("请输入一条不超过 128 字符的 ASCII 指令，不要包含换行")

        if command == "#GETJPOS":
            values = self.read_joints()
            return {"command": command, "reply": "okj " + " ".join(f"{v:.2f}" for v in values)}
        if command == "#GETLPOS":
            values = self.read_pose()
            return {"command": command, "reply": "okl " + " ".join(f"{v:.2f}" for v in values)}
        if command == "!START":
            if self.busy:
                raise RuntimeError("Homing/Resting 正在执行，当前不能改变使能状态")
            return {"command": command, **self.set_enabled(True)}
        if command == "!DISABLE":
            if self.busy:
                raise RuntimeError("Homing/Resting 正在执行；如需中止请发送 !STOP")
            return {"command": command, **self.set_enabled(False)}
        if command == "!STOP":
            return {"command": command, **self.stop()}

        match = re.fullmatch(r"#GRIP\s+(" + FLOAT_TOKEN + r")", command)
        if match:
            if not self.enabled:
                raise RuntimeError("发送夹爪指令前请先使能机械臂")
            angle = float(match.group(1))
            result = self.set_gripper(angle)
            return {"command": command, **result}

        match = re.fullmatch(r"#GRIP_CURRENT\s+(" + FLOAT_TOKEN + r")", command)
        if match:
            if not self.enabled:
                raise RuntimeError("设置夹爪电流前请先使能机械臂")
            if self.busy:
                raise RuntimeError("Homing/Resting 正在执行，当前不能设置夹爪电流")
            current = float(match.group(1))
            if not 0.0 <= current <= 1.0:
                raise ValueError("夹爪电流参数范围为 0 到 1")
            reply = self.command(command, lambda line: "Grip current" in line)
            return {"command": command, "reply": reply}

        match = re.fullmatch(r"#CMDMODE\s+([1-3])", command)
        if match:
            if self.busy:
                raise RuntimeError("Homing/Resting 正在执行，当前不能切换命令模式")
            mode = int(match.group(1))
            # This firmware branch changes the mode but does not send a reply.
            with self.lock:
                self.device.reset_input_buffer()
                self.device.write((command + "\r\n").encode("ascii"))
                self.device.flush()
            names = {1: "顺序目标点", 2: "可中断目标点", 3: "电机调试"}
            return {"command": command, "reply": f"已发送，固件不返回确认；模式请求：{names[mode]}"}

        if command[0] in ">&@":
            values_text = command[1:].split(",")
            expected_counts = {"&": (7, 8, 9), ">": (7,), "@": (7,)}
            if len(values_text) not in expected_counts[command[0]]:
                if command[0] == "&":
                    raise ValueError("& 指令格式：&J1,J2,J3,J4,J5,J6,速度[,夹爪角[,夹爪电流]]")
                raise ValueError(f"{command[0]} 指令格式：{command[0]}六个目标值,速度")
            try:
                values = [float(value) for value in values_text]
            except ValueError as exc:
                raise ValueError("运动指令中的参数必须是数字") from exc
            if not all(map(lambda value: value == value and abs(value) != float("inf"), values)):
                raise ValueError("运动指令中的参数必须是有限数值")
            if not self.enabled:
                raise RuntimeError("发送运动指令前请先使能机械臂")
            if self.busy:
                raise RuntimeError("Homing/Resting 正在执行，当前不能发送运动指令")
            joint_limits = CONFIG["robot"]["joints"]
            if command[0] in "&>":
                for index, (angle, limit) in enumerate(zip(values[:6], joint_limits), 1):
                    if not limit["min_deg"] <= angle <= limit["max_deg"]:
                        raise ValueError(f"J{index}={angle:g}° 超出范围 [{limit['min_deg']}, {limit['max_deg']}]°")
            speed = float(CONFIG["robot"]["joint_speed_deg_s"])
            if abs(values[6] - speed) > 1e-6:
                raise ValueError(f"运动速度必须为配置值 {speed:g}°/s")
            if command[0] == "&" and len(values) >= 8:
                gripper = CONFIG["robot"]["gripper"]
                if not gripper["min_deg"] <= values[7] <= gripper["max_deg"]:
                    raise ValueError(f"夹爪角需在 [{gripper['min_deg']}, {gripper['max_deg']}]°")
            if command[0] == "&" and len(values) == 9 and not 0.0 <= values[8] <= 1.0:
                raise ValueError("夹爪电流参数范围为 0 到 1")
            reply = self.command(command, lambda line: line == "ok" or line.isdigit())
            note = "固件已接收指令；串口回包不代表机械臂已到位"
            if reply.isdigit():
                note = f"指令已进入运动队列（剩余空间 {reply}）；请看实际遥测确认到位"
            return {"command": command, "reply": reply, "note": note}

        raise ValueError("不支持该指令。请使用 README 中列出的 USB-CDC ASCII 指令")


class CameraLink:
    def __init__(self):
        self.lock = threading.RLock()
        self.pipeline = None
        self.align = None
        self.connected = False
        self.error = ""

    def connect(self):
        with self.lock:
            if self.connected:
                return {"connected": True}
            pipeline = rs.pipeline()
            config = rs.config()
            camera = CONFIG["camera"]
            config.enable_stream(rs.stream.color, int(camera["width"]), int(camera["height"]),
                                 rs.format.bgr8, int(camera["fps"]))
            config.enable_stream(rs.stream.depth, int(camera["width"]), int(camera["height"]),
                                 rs.format.z16, int(camera["fps"]))
            profile = pipeline.start(config)
            self.pipeline = pipeline
            self.align = rs.align(rs.stream.color)
            self.connected = True
            self.error = ""
            self.name = profile.get_device().get_info(rs.camera_info.name)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.intrinsics = {"width": intr.width, "height": intr.height,
                               "fx": intr.fx, "fy": intr.fy,
                               "cx": intr.ppx, "cy": intr.ppy,
                               "depth_scale": self.depth_scale}
            return {"connected": True, "name": self.name, "intrinsics": self.intrinsics}

    def disconnect(self):
        with self.lock:
            if self.pipeline is not None:
                self.pipeline.stop()
            self.pipeline = None
            self.align = None
            self.connected = False
            return {"connected": False}

    def frame_jpeg(self):
        with self.lock:
            if not self.connected or self.pipeline is None:
                return None
            frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=2000))
            color = frames.get_color_frame()
            if not color:
                return None
            ok, encoded = cv2.imencode(".jpg", np.asanyarray(color.get_data()),
                                       [cv2.IMWRITE_JPEG_QUALITY, int(CONFIG["camera"]["jpeg_quality"])])
            return encoded.tobytes() if ok else None


ROBOT = RobotLink()
CAMERA = CameraLink()


def json_bytes(value):
    return json.dumps(value, ensure_ascii=False).encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    server_version = "DummyWeb/1.0"

    def log_message(self, fmt, *args):
        print("[DummyWeb]", self.address_string(), fmt % args)

    def send_json(self, value, status=200):
        body = json_bytes(value)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def body_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/status":
            try:
                self.send_json({"robot": ROBOT.status(), "camera": {"connected": CAMERA.connected,
                                  "name": getattr(CAMERA, "name", ""),
                                  "intrinsics": getattr(CAMERA, "intrinsics", None)},
                                "config": {"serial": CONFIG["serial"], "robot": CONFIG["robot"],
                                           "camera": CONFIG["camera"]}})
            except Exception as exc:
                self.send_json({"error": str(exc)}, 500)
            return
        if parsed.path == "/api/serial/scan":
            self.send_json({"ports": ROBOT.scan()})
            return
        if parsed.path == "/api/model":
            kind = urllib.parse.parse_qs(parsed.query).get("format", ["urdf"])[0].lower()
            if kind not in ("urdf", "mjcf"):
                self.send_json({"error": "format must be urdf or mjcf"}, 400)
                return
            try:
                path = Path(CONFIG["models"][kind]).resolve()
                model = parse_urdf(path) if kind == "urdf" else parse_mjcf(path)
                self.send_json(model)
            except Exception as exc:
                self.send_json({"error": str(exc)}, 500)
            return
        if parsed.path.startswith("/api/assets/"):
            filename = Path(parsed.path.rsplit("/", 1)[-1]).name
            path = (Path(CONFIG["models"]["mesh_dir"]).resolve() / filename).resolve()
            mesh_root = Path(CONFIG["models"]["mesh_dir"]).resolve()
            if path.parent != mesh_root or not path.is_file():
                self.send_error(404)
                return
            content = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "model/stl")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "public, max-age=3600")
            self.end_headers()
            self.wfile.write(content)
            return
        if parsed.path == "/api/camera/stream":
            if not CAMERA.connected:
                self.send_error(503, "Camera is disconnected")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while CAMERA.connected:
                    frame = CAMERA.frame_jpeg()
                    if frame:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                    time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError, OSError, RuntimeError):
                pass
            return
        relative = "index.html" if parsed.path == "/" else parsed.path.lstrip("/")
        path = (STATIC_DIR / relative).resolve()
        if STATIC_DIR.resolve() not in path.parents and path != STATIC_DIR.resolve() / "index.html":
            self.send_error(404)
            return
        if not path.is_file():
            self.send_error(404)
            return
        content = path.read_bytes()
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if content_type.startswith("text/") or content_type in ("application/javascript",):
            content_type += "; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        try:
            data = self.body_json()
            if parsed.path == "/api/serial/connect":
                result = ROBOT.connect(data)
            elif parsed.path == "/api/serial/disconnect":
                result = ROBOT.disconnect()
            elif parsed.path == "/api/robot/enable":
                result = ROBOT.set_enabled(bool(data.get("enabled")))
            elif parsed.path == "/api/robot/joints":
                result = ROBOT.move_joints(data.get("angles", []))
            elif parsed.path == "/api/robot/gripper":
                result = ROBOT.set_gripper(data.get("angle"))
            elif parsed.path in ("/api/robot/homing", "/api/robot/resting"):
                result = ROBOT.operation(parsed.path.rsplit("/", 1)[-1])
            elif parsed.path == "/api/robot/stop":
                result = ROBOT.stop()
            elif parsed.path == "/api/robot/command":
                result = ROBOT.send_user_command(data.get("command", ""))
            elif parsed.path == "/api/camera/connect":
                result = CAMERA.connect()
            elif parsed.path == "/api/camera/disconnect":
                result = CAMERA.disconnect()
            else:
                self.send_json({"error": "endpoint not found"}, 404)
                return
            self.send_json({"ok": True, **result})
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, 400)


def main():
    host = CONFIG["server"]["host"]
    port = int(CONFIG["server"]["port"])
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    print(f"DummyWeb listening on http://{host}:{port}")
    print(f"Config: {CONFIG_PATH}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping DummyWeb")
    finally:
        ROBOT.disconnect()
        CAMERA.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
