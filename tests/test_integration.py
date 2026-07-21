#!/usr/bin/env python3
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
BIN = ROOT / "build" / "autonomy_controller"

def run(*args):
    return subprocess.run([str(BIN), *map(str, args)], cwd=ROOT, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

def main():
    (ROOT / "runs").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=ROOT / "runs") as td:
        result = run("--run-dir", td)
        assert result.returncode == 0, result.stdout
        assert "MISSION_COMPLETE" in result.stdout
        decisions = pathlib.Path(td, "decisions.csv").read_text()
        assert "TAKEOFF" in decisions and "HOVER_STABILIZE" in decisions
        assert "TAKEOFF -> EXPLORE" not in decisions

    with tempfile.TemporaryDirectory(dir=ROOT / "runs") as td:
        result = run("--explore", "--run-dir", td)
        assert result.returncode == 0, result.stdout
        assert "MISSION_COMPLETE" in result.stdout

    source = ROOT / "log" / "3_frontierTest_scanlog.bin"
    outputs = []
    for _ in range(2):
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as td:
            result = run("--backend", "replay", "--input", source, "--run-dir", td)
            assert result.returncode == 0, result.stdout
            assert "serial_opened=no commands_sent=no" in result.stdout
            outputs.append(pathlib.Path(td, "decisions.csv").read_bytes())
    assert outputs[0] == outputs[1], "replay decisions are not deterministic"

    with tempfile.TemporaryDirectory() as td:
        truncated = pathlib.Path(td, "truncated.bin")
        data = source.read_bytes()
        truncated.write_bytes(data[:7 + 642 + 19])
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as rd:
            result = run("--backend", "replay", "--input", truncated, "--run-dir", rd)
            assert result.returncode == 0, result.stdout
            assert "warnings=1" in result.stdout
        wrong = pathlib.Path(td, "wrong.bin")
        wrong.write_bytes(b"SCLOG1\n" + bytes(642))
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as rd:
            result = run("--backend", "replay", "--input", wrong, "--run-dir", rd)
            assert result.returncode != 0 and "rejected" in result.stdout

    with tempfile.TemporaryDirectory(dir=ROOT / "runs") as td:
        result = run("--backend", "mavlink", "--allow-live-serial", "--serial",
                     "/dev/definitely-not-opened", "--map-source", "live", "--run-dir", td)
        assert result.returncode != 0
        assert "production map feed" in result.stdout

    makefile = (ROOT / "Makefile").read_text()
    launcher = (ROOT / "scripts" / "run_autonomy.sh").read_text()
    legacy = (ROOT / "c" / "frontier.c").read_text()
    assert "c/autonomy/autonomy_main.c" in makefile
    assert "build/autonomy_controller" in launcher
    assert "--allow-legacy-unsafe-frontier" in legacy
    print("integration tests: PASS")

if __name__ == "__main__":
    main()
