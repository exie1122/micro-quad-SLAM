#!/usr/bin/env python3
import json
import inspect
import struct
import sys
import tempfile
import unittest
import warnings
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
import plot_scan


def record(timestamp_ms=1000, armed=1, grid=None):
    values = [0x334E4353, timestamp_ms, 5, 0]
    values.extend([0.0] * 19)
    values.extend([0, 0, 0])
    values.extend([10, 20, 30, 40, 50, 0])
    values.extend([0, 0, 0, 0, armed, 0, 0, 0])
    values.append(512)
    values.append(grid if grid is not None else b"\0" * 512)
    return struct.pack(plot_scan.SCAN_FMT, *values)


class PlotScanParserTests(unittest.TestCase):
    def test_version_and_headerless_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            wrong = Path(directory) / "wrong.bin"
            wrong.write_bytes(b"SCLOG1\n" + record())
            with self.assertRaises(ValueError):
                plot_scan.parse_log(wrong)
            headerless = Path(directory) / "headerless.bin"
            headerless.write_bytes(record())
            with self.assertRaises(ValueError):
                plot_scan.parse_log(headerless)
            with warnings.catch_warnings(record=True):
                self.assertEqual(len(plot_scan.parse_log(headerless, allow_headerless=True)), 1)

    def test_corrupt_gap_resynchronizes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gap.bin"
            path.write_bytes(b"SCLOG3\n" + record(1000) + b"bad-gap" + record(1100))
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                records = plot_scan.parse_log(path)
            self.assertEqual([item["t_ms"] for item in records], [1000, 1100])
            self.assertTrue(any("skipped" in str(item.message) for item in caught))

    def test_known_corrupt_corpus_log_recovers_expected_records(self):
        with warnings.catch_warnings(record=True):
            records = plot_scan.parse_log(ROOT / "log" / "changedEnvironment_scanlog.bin")
        self.assertEqual(len(records), 234)
        self.assertEqual(max(item["session_id"] for item in records), 2)

    def test_timestamp_reset_summary_never_goes_negative(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sessions.bin"
            path.write_bytes(b"SCLOG3\n" + b"".join(record(t) for t in (1000, 1100, 50, 150)))
            records = plot_scan.parse_log(path)
            profile = plot_scan.build_filter_profile(0.0, use_startup_filter=False)
            summary = plot_scan.compute_session_summary(records, profile)
            self.assertEqual(summary["session_count"], 2)
            self.assertAlmostEqual(summary["duration_s"], 0.2)
            self.assertAlmostEqual(summary["armed_s"], 0.2)

    def test_little_endian_tof_decode(self):
        grid = bytearray(b"\xff\xff" * 256)
        grid[:2] = (1000).to_bytes(2, "little")
        decoded = plot_scan._decode_scan_record(record(grid=bytes(grid)), 0)
        _, _, _, rows, columns = plot_scan.build_projection_tables()
        measurements = plot_scan.extract_frame_measurements(
            decoded, rows, columns, 0.1, 2.0, row_filter_indices=set(range(8))
        )
        self.assertEqual(int(measurements[0][0]), 1000)


class PlotScanExportTests(unittest.TestCase):
    def test_export_is_unique_traceable_and_truthfully_named(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.bin"
            source.write_bytes(b"SCLOG3\n" + record())
            first = plot_scan.export_poster_images(
                np.array([0.0, 0.01, 0.02, 1.0, 1.01, 1.02]),
                np.array([0.0, 0.01, 0.02, 1.0, 1.01, 1.02]),
                0.1,
                directory,
                source_log_path=source,
                export_metadata={"test": True},
            )
            second = plot_scan.export_poster_images(
                np.array([0.0, 0.01, 0.02]),
                np.array([0.0, 0.01, 0.02]),
                0.1,
                directory,
                source_log_path=source,
            )
            self.assertNotEqual(first, second)
            manifest = json.loads((first / "manifest.json").read_text())
            self.assertIn("not proven free space", manifest["interpretation"])
            self.assertTrue(all(item["path"].startswith("tof_hit_density_") for item in manifest["outputs"]))
            self.assertEqual(manifest["source_log_sha256"], plot_scan.sha256_file(source))


class PlotScanInteractionTests(unittest.TestCase):
    def test_all_interactive_sliders_support_mouse_dragging(self):
        source = inspect.getsource(plot_scan.plot_data)
        self.assertNotIn("dragging=False", source)
        self.assertGreaterEqual(source.count("dragging=True"), 6)


if __name__ == "__main__":
    unittest.main()
