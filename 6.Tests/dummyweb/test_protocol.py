"""Serial protocol regressions; no serial port or camera is opened."""
import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("dummyweb_server", ROOT / "3.Software/DummyWeb/server.py")
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class FakeSerial:
    is_open = True

    def __init__(self):
        self.writes = []
        self.lines = []
        self.responses = {
            "#GETSTATUS": ["oks 0 2 0 0 0 0 0 0 0"],
            "#GETJPOS": ["okj 0 -75 180 0 0 0"],
            "#GETLPOS": ["okl 220 0 300 0 90 0"],
            "#CMDMODE 2": ["Set command mode to [2]"],
            "#CMDMODE 4": ["Set command mode to [4]"],
            "!START": ["Started ok"], "!STOP": ["Stopped ok"],
            "!DISABLE": ["Disabled ok"], "!CLEARFAULT": ["Faults cleared ok"],
        }

    def reset_input_buffer(self):
        self.lines.clear()

    def write(self, data):
        command = data.decode("ascii").strip()
        self.writes.append(command)
        # Include a stale completion: it must not count as the queue acknowledgement.
        self.lines = list(self.responses.get(command, ["ok", "15"]))
        return len(data)

    def flush(self):
        raise AssertionError("Do not call unbounded tcdrain on the control path")

    def close(self):
        self.is_open = False

    def readline(self):
        return ((self.lines.pop(0) + "\r\n").encode("ascii") if self.lines else b"")


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.link = server.RobotLink()
        self.link.device = FakeSerial()
        self.link.enabled = True

    def test_slider_uses_int_queue_and_preserves_actual_feedback(self):
        before = self.link.last_joints.copy()
        result = self.link.move_joints([0, -70, 175, 0, 0, 0])
        self.assertEqual(self.link.device.writes[0], "#CMDMODE 2")
        self.assertTrue(self.link.device.writes[1].startswith(">0.00,-70.00,175.00"))
        self.assertEqual(result["reply"], "15")
        self.assertEqual(self.link.last_joints, before)

    def test_old_prefix_and_invalid_extensions_are_rejected(self):
        for command in ["&0,-75,180,0,0,0,30", ">0,-75,180,0,0,0,30,46",
                        ">0,-75,180,0,0,0,30,40,2", ">nan,-75,180,0,0,0,30",
                        ">0,-75,180,0,0,0,30,40,0.5,9", "@0,0,0,0,0,0,30,40"]:
            with self.subTest(command=command), self.assertRaises(ValueError):
                self.link.send_user_command(command)
        self.assertEqual(self.link.device.writes, [])

    def test_new_gripper_extensions_and_tuning_mode(self):
        self.assertEqual(self.link.send_user_command(">0,-75,180,0,0,0,30,40,0.5")["reply"], "15")
        self.assertEqual(self.link.send_user_command("#CMDMODE 4")["reply"], "Set command mode to [4]")

    def test_errors_and_legacy_queue_full_are_never_success(self):
        for reply in ["error robot disabled", "255"]:
            self.link.device.responses["!START"] = [reply]
            with self.subTest(reply=reply), self.assertRaises(RuntimeError):
                self.link.set_enabled(True)

    def test_fault_status_survives_successful_position_queries(self):
        self.link.device.responses["#GETSTATUS"] = ["oks 0 2 6 0 1 8 0 0 0"]
        status = self.link.status()
        self.assertFalse(status["enabled"])
        self.assertEqual(status["fault_mask"], 6)
        self.assertIn("J2: 过载", status["error"])
        self.assertIn("J3: CAN反馈超时", status["error"])
        with self.assertRaises(RuntimeError):
            self.link.set_enabled(True)
        self.assertNotIn("!START", self.link.device.writes)

    def test_clear_never_enables_or_replays_motion(self):
        self.link.enabled = False
        self.link.faults = [0, 1, 0, 0, 0, 0]
        result = self.link.clear_faults()
        self.assertFalse(result["enabled"])
        self.assertEqual(self.link.device.writes, ["!CLEARFAULT", "#GETSTATUS"])

    def test_can_diagnostics_are_read_only_while_faulted(self):
        self.link.enabled = False
        self.link.faults = [8] * 6
        device = self.link.device
        device.responses["#GETCAN"] = ["okc 1200 1201 0 0 0 0"]
        device.responses["#GETDRV 2"] = ["okd 1 20 1 0 8 251 100", "okd 2 10 1 0 8 251 105"]
        self.assertEqual(self.link.send_user_command("#GETCAN")["reply"], "okc 1200 1201 0 0 0 0")
        self.assertEqual(self.link.send_user_command("#GETDRV 2")["reply"], "okd 2 10 1 0 8 251 105")
        self.assertEqual(device.writes, ["#GETCAN", "#GETDRV 2"])
        self.assertFalse(self.link.enabled)
        device.responses["#GETTIMING 2"] = ["okt 2 1200 2000 3"]
        self.assertEqual(self.link.send_user_command("#GETTIMING 2")["reply"], "okt 2 1200 2000 3")
        self.link.faults = [0,128,0,0,0,0]
        self.assertEqual(self.link.fault_message(), "J2: F1控制循环超时")
        for command in ["#GETTIMING 0", "#GETTIMING 7", "#GETDRV 0", "#GETDRV 7", "#GETDRV 2 extra"]:
            with self.assertRaises(ValueError):
                self.link.send_user_command(command)

    def test_joined_ack_is_not_guessed_or_replayed(self):
        device = self.link.device
        command = ">0,0,90,0,0,0,30"
        device.responses[command] = ["15ok"]
        with patch.object(server.time, "monotonic", side_effect=[0, 0, 0, 3]):
            with self.assertRaisesRegex(TimeoutError, "15ok"):
                self.link.command(command, self.link.queue_reply, timeout=2)
        self.assertEqual(device.writes, [command])
        self.assertFalse(device.is_open)

    def test_stop_updates_enabled_state(self):
        self.assertFalse(self.link.stop()["enabled"])
        self.assertFalse(self.link.enabled)

    def test_status_failure_does_not_leave_ready(self):
        self.link.device.responses["#GETSTATUS"] = ["error unsupported command"]
        status = self.link.status()
        self.assertFalse(status["enabled"])
        self.assertIn("固件拒绝", status["error"])

    def test_connect_requires_firmware_handshake_and_allows_fault_diagnosis(self):
        device = FakeSerial()
        device.responses["#GETSTATUS"] = ["oks 0 2 37 8 0 8 0 0 8"]
        with patch.object(server.serial, "Serial", return_value=device), patch.object(server.time, "sleep"):
            result = self.link.connect({"port": "/dev/test"})
        self.assertTrue(result["connected"])
        self.assertEqual(device.writes, ["#GETSTATUS"])
        self.assertFalse(self.link.enabled)
        self.assertIn("J6: CAN反馈超时", self.link.last_error)

    def test_connect_without_status_never_reports_connected(self):
        device = FakeSerial()
        device.responses["#GETSTATUS"] = ["error unsupported command"]
        with patch.object(server.serial, "Serial", return_value=device), patch.object(server.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "状态握手失败"):
                self.link.connect({"port": "/dev/test"})
        self.assertFalse(self.link.connected)
        self.assertFalse(device.is_open)
        self.assertFalse(self.link.enabled)
        self.assertEqual(device.writes, ["#GETSTATUS"])

    def test_write_timeout_closes_connection_before_enable(self):
        device = self.link.device
        device.write = Mock(side_effect=server.serial.SerialTimeoutException("Write timeout"))
        with self.assertRaisesRegex(RuntimeError, r"写入失败（#GETSTATUS）"):
            self.link.set_enabled(True)
        device.write.assert_called_once_with(b"#GETSTATUS\r\n")
        self.assertFalse(device.is_open)
        self.assertFalse(self.link.connected)
        self.assertFalse(self.link.enabled)
        self.assertFalse(self.link.status()["connected"])

    def test_short_write_is_not_retried(self):
        device = self.link.device
        device.write = Mock(return_value=3)
        with self.assertRaisesRegex(RuntimeError, "仅写入"):
            self.link.command("!START")
        device.write.assert_called_once()
        self.assertFalse(device.is_open)

    def test_read_error_updates_status_connection_state(self):
        device = self.link.device
        device.readline = Mock(side_effect=server.serial.SerialException("device disconnected"))
        result = self.link.status()
        self.assertFalse(result["connected"])
        self.assertFalse(result["enabled"])
        self.assertIn("串口读取失败", result["error"])
        self.assertFalse(device.is_open)

    def test_firmware_assert_is_reported_immediately_and_closes_link(self):
        device = self.link.device
        device.responses["#GETJPOS"] = ['assertion "Balloc succeeded" failed: file "mprec.c", line 783']
        with self.assertRaisesRegex(RuntimeError, 'F4 固件触发运行库断言.*Balloc succeeded'):
            self.link.read_joints()
        self.assertEqual(device.writes, ["#GETJPOS"])
        self.assertFalse(device.is_open)
        self.assertFalse(self.link.connected)

    def test_missing_motion_ack_closes_without_replaying(self):
        device = self.link.device
        device.responses["!START"] = []
        with patch.object(server.time, "monotonic", side_effect=[0, 0, 3]):
            with self.assertRaisesRegex(TimeoutError, "F4 回包超时"):
                self.link.command("!START", lambda value: value == "Started ok", timeout=2)
        self.assertEqual(device.writes, ["!START"])
        self.assertFalse(device.is_open)
        self.assertFalse(self.link.enabled)

    def test_clear_http_route(self):
        handler = server.Handler.__new__(server.Handler)
        handler.path = "/api/robot/clear_faults"
        handler.body_json = Mock(return_value={})
        handler.send_json = Mock()
        with patch.object(server.ROBOT, "clear_faults", return_value={"enabled": False}) as clear:
            handler.do_POST()
        clear.assert_called_once()
        handler.send_json.assert_called_once_with({"ok": True, "enabled": False})


if __name__ == "__main__":
    unittest.main()
