"""Explicit hardware test of DummyWeb's read-only telemetry path.

Run manually with the DummyRobot Python environment after flashing F4.
Only telemetry queries are sent; no enable, clear-fault or motion commands.
--can-diagnostics adds #GETCAN and #GETDRV 1..6 (requires the matching F4).
--timing adds #GETTIMING 1..6 (requires matching F1 and F4).
"""
import argparse
import importlib.util
import math
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--cycles", type=int, default=200)
    parser.add_argument("--can-diagnostics", action="store_true")
    parser.add_argument("--timing", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.cycles <= 10000:
        parser.error("cycles must be between 1 and 10000")
    sys.dont_write_bytecode = True
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location("dummyweb_probe", root / "3.Software/DummyWeb/server.py")
    server = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(server)
    link = server.RobotLink()
    started = time.monotonic()
    max_roundtrip = 0
    diagnostic_queries = 0
    try:
        link.connect({"port": args.port})
        if link.enabled:
            raise RuntimeError("F4 reports enabled; leave it unchanged and stop this idle validation")
        for cycle in range(1, args.cycles + 1):
            before = time.monotonic()
            state = link.read_status()
            state["joints_deg"] = link.read_joints()
            state["pose_mm_deg"] = link.read_pose()
            duration = time.monotonic() - before
            max_roundtrip = max(max_roundtrip, duration)
            if state["enabled"]:
                raise RuntimeError("F4 became enabled; stop the idle test without changing its state")
            values = state["joints_deg"] + state["pose_mm_deg"]
            if len(values) != 12 or not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"invalid telemetry: {values}")
            if cycle % 50 == 0 or cycle == args.cycles:
                print(f"{cycle}/{args.cycles} cycles OK; faults={state['faults']}; "
                      f"max status cycle={max_roundtrip:.3f}s", flush=True)
                if args.can_diagnostics:
                    for command in ["#GETCAN"] + [f"#GETDRV {joint}" for joint in range(1, 7)]:
                        print(link.send_user_command(command)["reply"], flush=True)
                        diagnostic_queries += 1
                if args.timing:
                    for joint in range(1, 7):
                        print(link.send_user_command(f"#GETTIMING {joint}")["reply"], flush=True)
                        diagnostic_queries += 1
            time.sleep(max(0, 0.1 - duration))
        print(f"PASS: {1 + 3 * args.cycles + diagnostic_queries} read-only queries in "
              f"{time.monotonic() - started:.1f}s", flush=True)
        print(f"joints={state['joints_deg']}; pose={state['pose_mm_deg']}", flush=True)
    finally:
        link.disconnect()


if __name__ == "__main__":
    main()
