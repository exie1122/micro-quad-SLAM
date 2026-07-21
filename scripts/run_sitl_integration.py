#!/usr/bin/env python3
"""Run the production C mission/backend against a real ArduCopter SITL process.

The ToF side uses a PTY carrying the exact legacy-a5-v0 production wire format;
it is not a fake map injected behind the planner. No physical serial port is used.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import pty
import signal
import subprocess
import tempfile
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]


def tof_frame(uptime_ms: int, range_mm: int = 1500) -> bytes:
    frame = bytearray(518)
    frame[0] = 0xA5
    frame[1:5] = (uptime_ms & 0xFFFFFFFF).to_bytes(4, "little")
    for zone in range(4 * 64):
        frame[5 + zone * 2:7 + zone * 2] = range_mm.to_bytes(2, "little")
    checksum = 0
    for value in frame[:-1]:
        checksum ^= value
    frame[-1] = checksum
    return bytes(frame)


def feed_tof(master_fd: int, stop: threading.Event, terminate: threading.Event) -> None:
    started = time.monotonic()
    while not terminate.is_set():
        if not stop.is_set():
            try:
                os.write(master_fd, tof_frame(int((time.monotonic() - started) * 1000)))
            except OSError:
                return
        time.sleep(0.08)


def terminate(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)


def run_case(ardupilot_root: pathlib.Path, fault: str, timeout_s: float) -> tuple[bool, str]:
    sitl = ardupilot_root / "build/sitl/bin/arducopter"
    defaults = [
        ardupilot_root / "Tools/autotest/default_params/copter.parm",
        ardupilot_root / "Tools/autotest/default_params/copter-optflow.parm",
        ardupilot_root / "Tools/autotest/default_params/copter-rangefinder.parm",
    ]
    if not sitl.is_file() or any(not item.is_file() for item in defaults):
        return False, f"missing built SITL or defaults under {ardupilot_root}"
    subprocess.run(["make", "all"], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    stop_feed, terminate_feed = threading.Event(), threading.Event()
    feeder = threading.Thread(target=feed_tof, args=(master_fd, stop_feed, terminate_feed), daemon=True)
    feeder.start()
    with tempfile.TemporaryDirectory(prefix="micro-quad-sitl-") as temp:
        temp_path = pathlib.Path(temp)
        run_dir = temp_path / "run"
        run_dir.mkdir()
        sitl_command = [
            str(sitl), "--model", "quad", "--wipe", "--speedup", "1",
            "--defaults", ",".join(map(str, defaults)),
            "--serial0", "udpclient:127.0.0.1:14551",
        ]
        with (temp_path / "sitl.log").open("w") as sitl_log:
            sitl_process = subprocess.Popen(
                sitl_command, cwd=temp_path, text=True, stdout=sitl_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
        autonomy_command = [
            str(ROOT / "build/autonomy_controller"),
            "--backend", "sitl", "--serial", "udpin:127.0.0.1:14551",
            "--expected-system", "1", "--expected-component", "1",
            "--tof-serial", slave_name, "--tof-baud", "921600",
            "--tof-protocol", "legacy-a5-v0", "--map-source", "live",
            # ArduPilot's built-in SITL optical-flow backend reports quality 51.
            "--minimum-flow-quality", "50",
            "--start-mission", "--explore", "--max-runtime-s", str(timeout_s),
            "--run-dir", str(run_dir),
        ]
        # Let the wiped FC finish its own initialization before the companion's
        # preflight dwell starts. The C backend still independently requires the
        # MAV_SYS_STATUS_PREARM_CHECK health bit when ArduPilot advertises it.
        time.sleep(5.0)
        autonomy_process = subprocess.Popen(
            autonomy_command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        fault_injected = False
        deadline = time.monotonic() + timeout_s + 25
        while autonomy_process.poll() is None and time.monotonic() < deadline:
            decisions = (run_dir / "decisions.csv")
            text = decisions.read_text(errors="replace") if decisions.exists() else ""
            if fault == "stale-tof" and "EXPLORE" in text and not fault_injected:
                stop_feed.set()
                fault_injected = True
            time.sleep(0.20)
        if autonomy_process.poll() is None:
            autonomy_process.send_signal(signal.SIGTERM)
        try:
            output, _ = autonomy_process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            autonomy_process.kill()
            output, _ = autonomy_process.communicate()
        terminate_feed.set()
        feeder.join(timeout=2)
        os.close(master_fd)
        os.close(slave_fd)
        terminate(sitl_process)
        decisions_text = (run_dir / "decisions.csv").read_text(errors="replace") if (run_dir / "decisions.csv").exists() else ""
        telemetry_text = (run_dir / "telemetry.csv").read_text(errors="replace") if (run_dir / "telemetry.csv").exists() else ""
        sitl_text = (temp_path / "sitl.log").read_text(errors="replace") if (temp_path / "sitl.log").exists() else ""
        required = ["SET_MODE", "ARM_REQUESTED", "TAKEOFF", "HOVER_STABILIZE", "HOVER_SCAN", "LAND", "DISARM"]
        if fault == "none":
            required.extend(["SELECT_FRONTIER", "PLAN_PATH", "EXPLORE"])
        else:
            required.extend(["EXPLORE", "HOLD"])
        missing = [state for state in required if state not in decisions_text]
        ok = autonomy_process.returncode == 0 and not missing and (fault == "none" or fault_injected)
        summary = (
            f"fault={fault} rc={autonomy_process.returncode} injected={fault_injected} "
            f"missing={','.join(missing) or 'none'}\nAUTONOMY:\n{output[-3000:]}\n"
            f"DECISIONS:\n{decisions_text[-2500:]}\nTELEMETRY:\n{telemetry_text[-2500:]}\nSITL:\n{sitl_text[-2500:]}"
        )
        return ok, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ardupilot-root", type=pathlib.Path, required=True)
    parser.add_argument("--fault", choices=("none", "stale-tof"), default="none")
    parser.add_argument("--timeout-s", type=float, default=40.0)
    args = parser.parse_args()
    ok, summary = run_case(args.ardupilot_root.resolve(), args.fault, args.timeout_s)
    print(summary)
    print("SITL integration:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
