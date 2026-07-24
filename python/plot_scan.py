import struct
import sys
import math
import argparse
import os
import hashlib
import json
import warnings
from datetime import datetime, timezone
from pathlib import Path
try:
    import tkinter as tk
    from tkinter import filedialog
except ImportError:
    tk = None

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider
except ImportError as e:
    print("Error: This script requires 'numpy' and 'matplotlib'.")
    print("Please install them with: pip install numpy matplotlib")
    sys.exit(1)

try:
    from scipy import ndimage as ndi
except ImportError:
    ndi = None

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
DEFAULT_LOG_FILE = "scanlog.bin"
GRID_SIZE_M = 20.0  # Approx size of plot in meters
MAX_DISPLAY_POINTS = 200000
MAX_TRAJECTORY_POINTS = 20000
OCCUPANCY_CELL_SIZE_M = 0.10
OCCUPANCY_SATURATION_HITS = 8.0
OCCUPANCY_MIN_HIT_THRESHOLD = 3
MIN_OCCUPANCY_CELL_SIZE_M = 0.05
MAX_OCCUPANCY_CELL_SIZE_M = 0.25
ROOM_FIT_MIN_POINTS = 80
ROOM_FIT_TRIM_PERCENTILE = 2.0
# Poster export / morphology parameters
POSTER_MORPH_CLOSING_SIZE = 3       # Square kernel for closing (bridges short gaps)
POSTER_MORPH_DILATE = False         # One light dilation pass for thin walls
POSTER_MORPH_DILATE_SIZE = 3        # Cross kernel size for dilation
POSTER_MIN_CLUSTER_CELLS = 4       # Remove connected components smaller than this
POSTER_CROP_MARGIN_M = 0.3         # Margin around occupied cells when cropping
# VL53L5CX config
NUM_SENSORS = 4
ZONES_PER_SENSOR = 64 # 8x8
ROWS = 8
COLS = 8
FOV_H = 45.0 # Degrees
FOV_V = 45.0 # Degrees
MIDDLE_ROWS_ONLY = True
MIDDLE_ROW_INDICES = {3, 4}
DEFAULT_ROW_MODE = "both"
# Logged sensor order is Front, Right, Back, Left.
SENSOR_NAMES = ["Front", "Right", "Back", "Left"]
SENSOR_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:purple"]
ENABLED_SENSOR_MASK = np.array([True, True, True, True], dtype=bool)
# Body-frame convention for sensor offsets:
#   +X = forward, +Y = right
# Approximate frame geometry from the repository. These values are not a complete
# measured 6-DoF sensor calibration and exports label them accordingly.
SENSOR_GEOMETRY_STATUS = "repository defaults; per-zone rays and 6-DoF extrinsics uncalibrated"
DEFAULT_SENSOR_OFFSETS_BODY_XY_M = np.array([
    [0.03, 0.0],   # Front
    [0.0, 0.03],   # Right
    [-0.03, 0.0],  # Back
    [0.0, -0.03],  # Left
], dtype=np.float32)
# Per-sensor mounting yaw trim in degrees. Positive rotates the sensor
# clockwise in this viewer's X/Y convention.
DEFAULT_SENSOR_YAW_TRIMS_DEG = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
DEFAULT_COLUMN_AZ_OFFSETS_DEG = np.zeros((NUM_SENSORS, COLS), dtype=np.float32)
OUTLIER_REJECT_RADIUS_M = 0.2 #lower = more aggressive
OUTLIER_REJECT_MIN_NEIGHBORS = 4 #higher = more aggressive
MANHATTAN_MIN_SENSOR_POINTS = 4
MANHATTAN_MIN_CONTRIBUTING_SENSORS = 2
MANHATTAN_MIN_LINEARITY = 3.0
MANHATTAN_MAX_CORRECTION_DEG = 25.0
MANHATTAN_CORRECTION_GAIN = 0.45
MANHATTAN_MAX_STEP_DEG = 6.0
DEFAULT_FILTER_AGGRESSIVENESS = 2.0
MAP_MIN_RANGE_M = 0.15
MAP_MAX_RANGE_M = 3.50
MANHATTAN_MIN_RANGE_M = 0.20
MANHATTAN_MAX_RANGE_M = 2.50
MIN_OF_QUALITY = 40
MAX_ATT_AGE_MS = 400
MAX_LPOS_AGE_MS = 400
MAX_OF_AGE_MS = 400
MAX_RF_AGE_MS = 400
MAX_HEARTBEAT_AGE_MS = 1500
MAX_ABS_TILT_RAD = 0.40
MIN_STABLE_ALT_M = 0.12
STABLE_HOVER_VZ_THRESH = 0.15
STABLE_HOVER_XY_SPEED_THRESH = 0.15
STABLE_HOVER_DWELL_MS = 500
POSEF_FC_LINK = 1 << 0
POSEF_FC_ARMED = 1 << 1
POSEF_ATT_FRESH = 1 << 2
POSEF_LPOS_FRESH = 1 << 3
POSEF_OF_FRESH = 1 << 4
POSEF_RF_FRESH = 1 << 5
POSEF_SYS_FRESH = 1 << 6
POSEF_ALT_VALID = 1 << 7
POSEF_HUMAN_SEEN = 1 << 8
REQUIRED_POSE_FLAGS = POSEF_FC_LINK | POSEF_ATT_FRESH | POSEF_LPOS_FRESH | POSEF_ALT_VALID

# -----------------------------------------------------------------------------
# Data Structures
# -----------------------------------------------------------------------------
# scanrec_t structure layout (packed, little-endian)
# See manual_uav_fc_tof_nav.c for definition
#
# uint32_t magic;       // 0
# uint64_t host_ms;     // 4
# uint32_t scan_ms;     // 12
# uint32_t custom_mode; // 16
# float x_m, y_m, yaw_deg, alt_m, alt_max_m; // 20 + 5*4
# float roll_rad, pitch_rad;                 // 40 + 2*4
# float vx_mps, vy_mps, vz_mps;              // 48 + 3*4
# float lpos_alt_m, lpos_alt_filt_m;         // 60 + 2*4
# float rf_m, rf_v;                          // 68 + 2*4
# float of_rate_x, of_rate_y;                // 76 + 2*4
# float of_comp_m_x, of_comp_m_y;            // 84 + 2*4
# float of_ground_m;                         // 92 + 4
# uint32_t sys_present, sys_health, sys_enabled; // 96 + 3*4 = 108
# uint16_t att_age, lpos_age, of_age, rf_age, hb_age, pose_flags; // 108 + 6*2 = 120
# uint8_t of_q, alt_src, rf_src, of_src, fc_armed, alt_rf_rej, ds_ori, ds_id; // 120 + 8*1 = 128
# uint16_t ds_cur_cm; // 128 + 2 = 130
# uint8_t grid[512];  // 130 + 512 = 642 bytes
#
RECORD_SIZE = 642
SCAN_FMT = '<IQIIfffffffffffffffffffIIIHHHHHHBBBBBBBBH512s'
SCAN_HEADER = b'SCLOG3\n'
SCAN_RECORD_MAGIC = b'SCN3'
SCAN_RECORD_MAGIC_U32 = 0x334E4353

if struct.calcsize(SCAN_FMT) != RECORD_SIZE:
    raise RuntimeError("SCLOG3 structure definition no longer matches its 642-byte contract")

# -----------------------------------------------------------------------------
# Parsing
# -----------------------------------------------------------------------------
def _decode_scan_record(chunk, file_offset):
    """Decode one already-aligned SCLOG3 record."""
    if len(chunk) != RECORD_SIZE or chunk[:4] != SCAN_RECORD_MAGIC:
        raise ValueError(f"invalid SCLOG3 record at byte {file_offset}")
    data = struct.unpack(SCAN_FMT, chunk)
    if data[0] != SCAN_RECORD_MAGIC_U32:
        raise ValueError(f"invalid SCLOG3 magic at byte {file_offset}")
    return {
        'record_offset': int(file_offset),
        't_ms': data[1],
        'scan_ms': data[2],
        'custom_mode': data[3],
        'x': data[4],
        'y': data[5],
        'yaw': data[6],
        'alt_max': data[8],
        'roll': data[9],
        'pitch': data[10],
        'alt': data[7],
        'vx': data[11],
        'vy': data[12],
        'vz': data[13],
        'att_age_ms': data[26],
        'lpos_age_ms': data[27],
        'of_age_ms': data[28],
        'rf_age_ms': data[29],
        'hb_age_ms': data[30],
        'pose_flags': data[31],
        'human_seen': 1 if (data[31] & POSEF_HUMAN_SEEN) else 0,
        'of_q': data[32],
        'alt_src': data[33],
        'rf_src': data[34],
        'of_src': data[35],
        'fc_armed': data[36],
        'grid_bytes': data[41],
    }


def _assign_session_ids(records):
    """Annotate timestamp-reset-separated sessions without altering timestamps."""
    session_id = 0
    session_start_ms = None
    previous_ms = None
    for rec in records:
        timestamp_ms = int(rec['t_ms'])
        if previous_ms is not None and timestamp_ms < previous_ms:
            session_id += 1
            session_start_ms = None
        if session_start_ms is None:
            session_start_ms = timestamp_ms
        rec['session_id'] = session_id
        rec['session_elapsed_s'] = max(0.0, (timestamp_ms - session_start_ms) * 0.001)
        previous_ms = timestamp_ms
    return session_id + 1 if records else 0


def parse_log(filename, allow_headerless=False):
    print(f"Reading {filename}...")
    records = []

    with open(filename, 'rb') as f:
        # Check for header.
        # AUDIT 2026-07-12 (P0 fix): fail CLOSED on a version mismatch instead of
        # silently re-reading the bytes as SCLOG3. Previously any non-SCLOG3 file
        # (e.g. the SCLOG1 208 MB scanlog.bin in logs/) was seek(0)'d and decoded
        # with the 642-byte SCLOG3 layout, producing hundreds of thousands of
        # garbage records with no error. SCAN_FMT/RECORD_SIZE describe SCLOG3 only.
        header = f.read(len(SCAN_HEADER))
        payload_offset = len(SCAN_HEADER)
        if header == SCAN_HEADER:
            print("Found SCLOG3 header.")
        elif header[:5] == b'SCLOG':
            ver = header.rstrip(b'\n').decode('latin1', 'replace')
            raise ValueError(
                f"{filename}: header is '{ver}' but this parser only decodes SCLOG3 "
                f"(RECORD_SIZE={RECORD_SIZE}). Refusing to misinterpret the data. "
                f"Convert the log or use a version-matched reader."
            )
        elif allow_headerless:
            warnings.warn(
                f"{filename}: explicitly treating headerless bytes as SCLOG3 records.",
                RuntimeWarning,
                stacklevel=2,
            )
            f.seek(0)
            payload_offset = 0
        else:
            raise ValueError(
                f"{filename}: missing SCLOG3 header; use --allow-headerless-sclog3 "
                "only for a separately verified legacy headerless file."
            )

        # Read in bounded chunks and retain only enough data for resynchronization.
        buffer = bytearray()
        buffer_offset = payload_offset
        skipped_since_warning = 0
        while True:
            incoming = f.read(64 * 1024)
            if incoming:
                buffer.extend(incoming)
            eof = not incoming

            while len(buffer) >= 4:
                if buffer[:4] != SCAN_RECORD_MAGIC:
                    next_magic = buffer.find(SCAN_RECORD_MAGIC, 1)
                    if next_magic < 0:
                        if eof:
                            skipped_since_warning += len(buffer)
                            buffer_offset += len(buffer)
                            buffer.clear()
                        else:
                            discard = max(0, len(buffer) - 3)
                            skipped_since_warning += discard
                            buffer_offset += discard
                            del buffer[:discard]
                        break
                    skipped_since_warning += next_magic
                    buffer_offset += next_magic
                    del buffer[:next_magic]

                if len(buffer) < RECORD_SIZE:
                    break
                if skipped_since_warning:
                    warnings.warn(
                        f"{filename}: skipped {skipped_since_warning} corrupt/non-record "
                        f"byte(s) before byte {buffer_offset}.",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                    skipped_since_warning = 0
                records.append(_decode_scan_record(bytes(buffer[:RECORD_SIZE]), buffer_offset))
                del buffer[:RECORD_SIZE]
                buffer_offset += RECORD_SIZE

            if eof:
                break

        trailing = skipped_since_warning + len(buffer)
        if trailing:
            warnings.warn(
                f"{filename}: {trailing} trailing/corrupt byte(s) dropped.",
                RuntimeWarning,
                stacklevel=2,
            )

    session_count = _assign_session_ids(records)
            
    print(f"Loaded {len(records)} records across {session_count} session(s).")
    return records


def wrap_deg180(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def nearest_manhattan_axis_deg(angle_deg):
    return 90.0 * round(angle_deg / 90.0)


def estimate_alignment_angle_deg(px, py):
    """Estimate rotation (degrees) to align dominant wall direction with plot axes.

    Builds a histogram of displacement angles (mod 90 deg) between nearby points
    to find the dominant orientation, then returns the smallest rotation that
    brings it to the nearest axis.
    """
    if len(px) < 50:
        return 0.0

    rng = np.random.default_rng(42)
    n = len(px)
    if n > 4000:
        idx = rng.choice(n, 4000, replace=False)
        sx, sy = px[idx].astype(np.float64), py[idx].astype(np.float64)
    else:
        sx, sy = np.asarray(px, dtype=np.float64), np.asarray(py, dtype=np.float64)
    n = len(sx)

    cell = 0.3
    inv_cell = 1.0 / cell
    grid = {}
    for i in range(n):
        key = (int(math.floor(sx[i] * inv_cell)), int(math.floor(sy[i] * inv_cell)))
        grid.setdefault(key, []).append(i)

    raw_angles = []
    min_d2 = 0.05 ** 2
    max_d2 = 0.50 ** 2
    for i in range(n):
        gx = int(math.floor(sx[i] * inv_cell))
        gy = int(math.floor(sy[i] * inv_cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((gx + dx, gy + dy), []):
                    if j <= i:
                        continue
                    ddx = sx[j] - sx[i]
                    ddy = sy[j] - sy[i]
                    d2 = ddx * ddx + ddy * ddy
                    if min_d2 < d2 < max_d2:
                        raw_angles.append(math.degrees(math.atan2(ddy, ddx)) % 90.0)

    if len(raw_angles) < 30:
        return 0.0

    angles = np.array(raw_angles)
    n_bins = 90
    hist, _ = np.histogram(angles, bins=np.linspace(0, 90, n_bins + 1))

    # Circular smoothing
    k = 3
    padded = np.concatenate([hist[-k:], hist, hist[:k]])
    kernel = np.ones(2 * k + 1) / (2 * k + 1)
    smoothed = np.convolve(padded, kernel, mode='same')[k:-k]

    peak_idx = int(np.argmax(smoothed))
    peak_angle = peak_idx + 0.5

    if peak_angle > 45.0:
        return 90.0 - peak_angle
    else:
        return -peak_angle


def rotate_2d(x, y, angle_deg):
    """Rotate 2D coordinate arrays by angle_deg (counter-clockwise)."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return x * c - y * s, x * s + y * c


def rotate_all_points(points_x, points_y, angle_deg, origin_xy=(0.0, 0.0)):
    """Rotate an entire point set around a shared origin."""
    px = np.asarray(points_x, dtype=np.float32)
    py = np.asarray(points_y, dtype=np.float32)
    if px.shape != py.shape:
        raise ValueError("points_x and points_y must have matching shapes")

    if len(px) == 0 or angle_deg == 0.0:
        return px.copy(), py.copy()

    origin_x, origin_y = origin_xy
    rotated_x, rotated_y = rotate_2d(px - origin_x, py - origin_y, angle_deg)
    return rotated_x + origin_x, rotated_y + origin_y


def build_room_outline_xy(width, length, center_x, center_y, yaw_deg=0.0):
    """Return closed XY polyline for a rotated room rectangle."""
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    half_w = float(width) * 0.5
    half_l = float(length) * 0.5
    corners_local = [
        (-half_w, -half_l),
        (half_w, -half_l),
        (half_w, half_l),
        (-half_w, half_l),
        (-half_w, -half_l),
    ]

    corners_world = []
    for lx, ly in corners_local:
        wx = float(center_x) + lx * cos_yaw - ly * sin_yaw
        wy = float(center_y) + lx * sin_yaw + ly * cos_yaw
        corners_world.append((wx, wy))
    return np.asarray(corners_world, dtype=np.float64)


def estimate_room_box(
    points_x,
    points_y,
    points_z=None,
    trim_percentile=ROOM_FIT_TRIM_PERCENTILE,
    min_points=ROOM_FIT_MIN_POINTS,
):
    """Estimate an oriented room box from a filtered point cloud.

    Uses the dominant Manhattan wall angle, then trims low/high percentiles to
    suppress isolated ToF outliers before measuring extents.
    """
    px = np.asarray(points_x, dtype=np.float32)
    py = np.asarray(points_y, dtype=np.float32)
    valid_xy = np.isfinite(px) & np.isfinite(py)
    px = px[valid_xy]
    py = py[valid_xy]

    if len(px) < int(min_points):
        return None

    trim_pct = float(np.clip(trim_percentile, 0.0, 25.0))
    align_angle_deg = estimate_alignment_angle_deg(px, py)
    rotated_x, rotated_y = rotate_all_points(px, py, align_angle_deg)
    low, high = trim_pct, 100.0 - trim_pct

    x_min, x_max = np.percentile(rotated_x, [low, high])
    y_min, y_max = np.percentile(rotated_y, [low, high])
    width = float(max(0.0, x_max - x_min))
    length = float(max(0.0, y_max - y_min))
    center_rx = 0.5 * float(x_min + x_max)
    center_ry = 0.5 * float(y_min + y_max)
    center_x, center_y = rotate_all_points(
        np.array([center_rx], dtype=np.float32),
        np.array([center_ry], dtype=np.float32),
        -align_angle_deg,
    )
    yaw_deg = wrap_deg180(-align_angle_deg)

    # Use a stable convention so width is the shorter horizontal side.
    if width > length:
        width, length = length, width
        yaw_deg = wrap_deg180(yaw_deg + 90.0)

    room = {
        'width': width,
        'length': length,
        'center_x': float(center_x[0]),
        'center_y': float(center_y[0]),
        'yaw_deg': yaw_deg,
        'trim_percentile': trim_pct,
        'point_count': int(len(px)),
        'color_idx': 0,
    }

    if points_z is not None:
        pz = np.asarray(points_z, dtype=np.float32)
        valid_z = valid_xy & np.isfinite(pz)
        pz = pz[valid_z]
        if len(pz) >= max(20, int(min_points) // 2):
            z_min, z_max = np.percentile(pz, [low, high])
            room['floor_z'] = float(z_min)
            room['height'] = float(max(0.0, z_max - z_min))
            room['ceiling_z'] = float(z_max)

    return room


def format_room_box_summary(room_box):
    """Compact human-readable room fit summary."""
    if not room_box:
        return "Room fit unavailable"

    if 'height' in room_box:
        dims = (
            f"{room_box['width']:.2f} x {room_box['length']:.2f} x "
            f"{room_box['height']:.2f} m"
        )
    else:
        dims = f"{room_box['width']:.2f} x {room_box['length']:.2f} m"

    parts = [
        dims,
        f"center=({room_box['center_x']:.2f}, {room_box['center_y']:.2f})",
        f"yaw={room_box['yaw_deg']:.1f} deg",
    ]
    if 'height' in room_box:
        parts.append(f"floor_z={room_box.get('floor_z', 0.0):.2f}")
    return " | ".join(parts)


def format_room_cli_args(room_box):
    """Return CLI arguments that recreate the current 2D/3D room box."""
    if not room_box:
        return ""

    parts = [
        f"--room {room_box['width']:.2f}x{room_box['length']:.2f}",
        f"--room-offset {room_box['center_x']:.2f},{room_box['center_y']:.2f}",
        f"--room-yaw {room_box.get('yaw_deg', 0.0):.1f}",
    ]
    if 'height' in room_box:
        parts[0] = (
            f"--room {room_box['width']:.2f}x{room_box['length']:.2f}x"
            f"{room_box['height']:.2f}"
        )
    return " ".join(parts)


def parse_matplotlib_key(key_value):
    """Normalize Matplotlib key strings across backends.

    Some backends report Shift+letter as 'J' while others report 'shift+j'.
    Return (base_key, is_shift).
    """
    if not key_value:
        return "", False

    raw_key = str(key_value)
    lowered = raw_key.lower()
    if lowered.startswith('shift+'):
        return lowered.split('+', 1)[1], True

    # Uppercase single letters usually indicate Shift was held.
    if len(raw_key) == 1 and raw_key.isalpha() and raw_key.isupper():
        return raw_key.lower(), True

    return lowered, False


def row_mode_to_indices(row_mode):
    if row_mode == "r3":
        return {3}
    if row_mode == "r4":
        return {4}
    return set(MIDDLE_ROW_INDICES)


def row_mode_label(row_mode):
    if row_mode == "r3":
        return "R3"
    if row_mode == "r4":
        return "R4"
    return "Both"


def next_row_mode(row_mode):
    if row_mode == "both":
        return "r3"
    if row_mode == "r3":
        return "r4"
    return "both"


def build_filter_profile(aggressiveness, use_startup_filter=True):
    a = float(np.clip(aggressiveness, 0.0, 2.0))
    return {
        'aggressiveness': a,
        'map_min_range_m': min(1.0, 0.05 + 0.10 * a),
        'map_max_range_m': max(2.0, 4.80 - 1.30 * a),
        'manhattan_min_range_m': min(1.0, 0.05 + 0.15 * a),
        'manhattan_max_range_m': max(1.5, 3.70 - 1.20 * a),
        'min_of_quality': int(round(10 + 30 * a)),
        'max_att_age_ms': int(round(580 - 180 * a)),
        'max_lpos_age_ms': int(round(580 - 180 * a)),
        'max_of_age_ms': int(round(650 - 250 * a)),
        'max_rf_age_ms': int(round(650 - 250 * a)),
        'max_heartbeat_age_ms': MAX_HEARTBEAT_AGE_MS,
        'max_abs_tilt_rad': max(0.18, 0.60 - 0.20 * a),
        'manhattan_min_linearity': 2.0 + 1.0 * a,
        'use_startup_filter': use_startup_filter,
        'stable_hover_t_ms': 0,
        'stable_hover_t_ms_by_session': {},
    }


def compute_stable_hover_t_ms(records):
    """Return the first session's stable-hover start for API compatibility."""
    starts = compute_stable_hover_t_ms_by_session(records)
    return starts.get(0, 0)


def compute_stable_hover_t_ms_by_session(records):
    """Find a measured stable-hover dwell independently for every session."""
    starts = {}
    candidate_start = None
    active_session = None
    for rec in records:
        session_id = int(rec.get('session_id', 0))
        if session_id != active_session:
            active_session = session_id
            candidate_start = None
        finite_motion = all(np.isfinite(rec[key]) for key in ('alt', 'vx', 'vy', 'vz'))
        stable = (
            finite_motion
            and rec['fc_armed'] != 0
            and rec['alt'] >= MIN_STABLE_ALT_M
            and math.hypot(rec['vx'], rec['vy']) < STABLE_HOVER_XY_SPEED_THRESH
            and abs(rec['vz']) < STABLE_HOVER_VZ_THRESH
        )
        if not stable:
            candidate_start = None
            continue
        if candidate_start is None:
            candidate_start = int(rec['t_ms'])
        if int(rec['t_ms']) - candidate_start >= STABLE_HOVER_DWELL_MS:
            starts.setdefault(session_id, candidate_start)
    return starts


def configure_startup_filter(filter_profile, records):
    starts = compute_stable_hover_t_ms_by_session(records)
    filter_profile['stable_hover_t_ms_by_session'] = starts
    filter_profile['stable_hover_t_ms'] = starts.get(0, 0)
    return filter_profile


def frame_pose_rejection_reason(rec, filter_profile):
    if not np.isfinite(rec['x']) or not np.isfinite(rec['y']) or not np.isfinite(rec['yaw']):
        return 'nonfinite_pose'
    if not np.isfinite(rec['roll']) or not np.isfinite(rec['pitch']):
        return 'nonfinite_attitude'
    if (rec['pose_flags'] & REQUIRED_POSE_FLAGS) != REQUIRED_POSE_FLAGS:
        return 'required_pose_flags'
    if rec['hb_age_ms'] > filter_profile['max_heartbeat_age_ms']:
        return 'stale_heartbeat'
    if rec['att_age_ms'] > filter_profile['max_att_age_ms']:
        return 'stale_attitude'
    if rec['lpos_age_ms'] > filter_profile['max_lpos_age_ms']:
        return 'stale_local_position'
    if (rec['pose_flags'] & POSEF_OF_FRESH) == 0:
        return 'optical_flow_not_fresh'
    if rec['of_age_ms'] > filter_profile['max_of_age_ms']:
        return 'stale_optical_flow'
    if rec['of_q'] < filter_profile['min_of_quality']:
        return 'poor_optical_flow_quality'
    if rec['alt_src'] in (2, 3):
        if (rec['pose_flags'] & POSEF_RF_FRESH) == 0:
            return 'range_not_fresh'
        if rec['rf_age_ms'] > filter_profile['max_rf_age_ms']:
            return 'stale_range'
    if abs(rec['roll']) > filter_profile['max_abs_tilt_rad']:
        return 'excess_roll'
    if abs(rec['pitch']) > filter_profile['max_abs_tilt_rad']:
        return 'excess_pitch'
    if filter_profile.get('use_startup_filter', True):
        stable_by_session = filter_profile.get('stable_hover_t_ms_by_session', {})
        stable_t = stable_by_session.get(
            int(rec.get('session_id', 0)),
            filter_profile.get('stable_hover_t_ms', 0),
        )
        if stable_t > 0 and rec['t_ms'] < stable_t:
            return 'before_stable_hover'
    return None


def frame_pose_is_usable(rec, filter_profile):
    return frame_pose_rejection_reason(rec, filter_profile) is None


def frame_rejection_counts(records, filter_profile):
    counts = {}
    for rec in records:
        reason = frame_pose_rejection_reason(rec, filter_profile) or 'accepted'
        counts[reason] = counts.get(reason, 0) + 1
    return counts


def extract_frame_measurements(
    rec,
    row_indices,
    col_indices,
    min_range_m,
    max_range_m,
    row_filter_indices=None,
):
    raw_dist = np.frombuffer(rec['grid_bytes'], dtype='<u2')
    min_mm = int(min_range_m * 1000.0)
    max_mm = int(max_range_m * 1000.0)
    valid_mask = (raw_dist >= min_mm) & (raw_dist <= max_mm)
    if not np.any(valid_mask):
        return None

    valid_indices = np.where(valid_mask)[0]
    dists_mm = raw_dist[valid_indices]
    sensor_idx = valid_indices // 64
    zone_idx_in_sensor = valid_indices % 64

    sensor_mask = ENABLED_SENSOR_MASK[sensor_idx]
    if not np.any(sensor_mask):
        return None
    dists_mm = dists_mm[sensor_mask]
    sensor_idx = sensor_idx[sensor_mask]
    zone_idx_in_sensor = zone_idx_in_sensor[sensor_mask]

    if row_filter_indices is None and MIDDLE_ROWS_ONLY:
        row_filter_indices = set(MIDDLE_ROW_INDICES)
    if row_filter_indices is not None:
        row_mask = np.isin(row_indices[zone_idx_in_sensor], list(row_filter_indices))
        if not np.any(row_mask):
            return None
        dists_mm = dists_mm[row_mask]
        sensor_idx = sensor_idx[row_mask]
        zone_idx_in_sensor = zone_idx_in_sensor[row_mask]

    return dists_mm, sensor_idx, zone_idx_in_sensor


def reject_isolated_points(px, py, sensor_idx, radius_m, min_neighbors):
    if len(px) == 0 or min_neighbors <= 0:
        return px, py, sensor_idx
    if len(px) <= min_neighbors:
        return np.array([], dtype=px.dtype), np.array([], dtype=py.dtype), np.array([], dtype=sensor_idx.dtype)

    pts = np.column_stack((px, py))
    deltas = pts[:, None, :] - pts[None, :, :]
    dist2 = np.sum(deltas * deltas, axis=2)
    neighbor_counts = np.count_nonzero(dist2 <= (radius_m * radius_m), axis=1) - 1
    keep_mask = neighbor_counts >= min_neighbors
    return px[keep_mask], py[keep_mask], sensor_idx[keep_mask]


def build_projection_tables():
    row_angles = (np.arange(ROWS, dtype=np.float32) - ((ROWS - 1) / 2.0)) * (FOV_V / ROWS)
    col_angles = (np.arange(COLS, dtype=np.float32) - ((COLS - 1) / 2.0)) * (FOV_H / COLS)

    elev_corrections = []
    elev_angles_rad = []
    azimuth_offsets = []
    row_indices = []
    col_indices = []

    for r in range(ROWS):
        elev_rad = math.radians(row_angles[r])
        c_elev = math.cos(elev_rad)
        for c in range(COLS):
            elev_corrections.append(c_elev)
            elev_angles_rad.append(elev_rad)
            azimuth_offsets.append(col_angles[c])
            row_indices.append(r)
            col_indices.append(c)

    return (
        np.array(elev_corrections, dtype=np.float32),
        np.array(elev_angles_rad, dtype=np.float32),
        np.array(azimuth_offsets, dtype=np.float32),
        np.array(row_indices, dtype=np.int16),
        np.array(col_indices, dtype=np.int16),
    )


def rotation_matrix_rz_ry_rx(yaw_rad, pitch_rad, roll_rad):
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float32)


def frame_body_points(
    rec,
    sensor_offsets_body_xy_m,
    sensor_yaw_trims_deg,
    column_az_offsets_deg,
    filter_profile,
    elev_corrections,
    azimuth_offsets,
    row_indices,
    col_indices,
    row_filter_indices=None,
):
    if not frame_pose_is_usable(rec, filter_profile):
        return None

    measurements = extract_frame_measurements(
        rec,
        row_indices,
        col_indices,
        filter_profile['manhattan_min_range_m'],
        filter_profile['manhattan_max_range_m'],
        row_filter_indices=row_filter_indices,
    )
    if measurements is None:
        return None

    dists_mm, sensor_idx, zone_idx_in_sensor = measurements
    dists_m = dists_mm / 1000.0
    planar_d = dists_m * elev_corrections[zone_idx_in_sensor]
    zone_col_idx = col_indices[zone_idx_in_sensor]
    corrected_azimuth_deg = (
        azimuth_offsets[zone_idx_in_sensor]
        + column_az_offsets_deg[sensor_idx, zone_col_idx]
    )
    body_angles_deg = (
        np.array([0.0, 90.0, 180.0, 270.0], dtype=np.float32)[sensor_idx]
        + sensor_yaw_trims_deg[sensor_idx]
        + corrected_azimuth_deg
    )
    body_angles_rad = np.radians(body_angles_deg)
    sensor_offsets_body = sensor_offsets_body_xy_m[sensor_idx]
    px_body = sensor_offsets_body[:, 0] + planar_d * np.cos(body_angles_rad)
    py_body = sensor_offsets_body[:, 1] + planar_d * np.sin(body_angles_rad)
    return px_body, py_body, sensor_idx


def estimate_manhattan_yaw_corrections(
    records,
    sensor_offsets_body_xy_m,
    sensor_yaw_trims_deg,
    column_az_offsets_deg,
    filter_profile,
    row_filter_indices=None,
):
    elev_corrections, _elev_angles_rad, azimuth_offsets, row_indices, col_indices = build_projection_tables()
    corrections_deg = np.zeros(len(records), dtype=np.float32)
    running_correction_deg = 0.0

    for rec_idx, rec in enumerate(records):
        frame = frame_body_points(
            rec,
            sensor_offsets_body_xy_m,
            sensor_yaw_trims_deg,
            column_az_offsets_deg,
            filter_profile,
            elev_corrections,
            azimuth_offsets,
            row_indices,
            col_indices,
            row_filter_indices=row_filter_indices,
        )
        if frame is None:
            continue

        px_body, py_body, sensor_idx = frame
        residuals = []
        weights = []
        contributing_sensors = 0

        for sid in range(NUM_SENSORS):
            if not ENABLED_SENSOR_MASK[sid]:
                continue
            mask = sensor_idx == sid
            if np.count_nonzero(mask) < MANHATTAN_MIN_SENSOR_POINTS:
                continue

            pts = np.column_stack((px_body[mask], py_body[mask]))
            pts_centered = pts - pts.mean(axis=0, keepdims=True)
            cov = pts_centered.T @ pts_centered
            eigvals, eigvecs = np.linalg.eigh(cov)
            if eigvals[0] <= 1e-9:
                continue
            linearity = float(eigvals[1] / eigvals[0])
            if linearity < filter_profile['manhattan_min_linearity']:
                continue
            principal = eigvecs[:, np.argmax(eigvals)]
            line_angle_body = math.degrees(math.atan2(principal[1], principal[0]))
            world_line_angle = rec['yaw'] + running_correction_deg + line_angle_body
            snapped = nearest_manhattan_axis_deg(world_line_angle)
            residual = wrap_deg180(snapped - world_line_angle)
            if abs(residual) > MANHATTAN_MAX_CORRECTION_DEG:
                continue
            residuals.append(residual)
            weights.append(float(np.max(eigvals)))
            contributing_sensors += 1

        if residuals and contributing_sensors >= MANHATTAN_MIN_CONTRIBUTING_SENSORS:
            weighted_residual_deg = np.average(
                np.array(residuals, dtype=np.float32),
                weights=np.array(weights, dtype=np.float32),
            )
            step_deg = float(np.clip(
                MANHATTAN_CORRECTION_GAIN * weighted_residual_deg,
                -MANHATTAN_MAX_STEP_DEG,
                MANHATTAN_MAX_STEP_DEG,
            ))
            running_correction_deg = wrap_deg180(running_correction_deg + step_deg)

        corrections_deg[rec_idx] = running_correction_deg

    return corrections_deg

# -----------------------------------------------------------------------------
# Point Cloud Generation
# -----------------------------------------------------------------------------
def compute_points(
    records,
    sensor_offsets_body_xy_m=None,
    sensor_yaw_trims_deg=None,
    column_az_offsets_deg=None,
    yaw_corrections_deg=None,
    use_3d_ray_rotation=True,
    filter_profile=None,
    row_filter_indices=None,
):
    """
    Convert raw ToF grid data + Pose into global 2D (x,y) points.
    In 3D mode, each zone is treated as a 3D ray in the drone body frame and
    rotated into the world frame using yaw, pitch, and roll before projecting
    onto XY. In planar mode, pitch/roll are ignored and only yaw is used.
    """
    print("Computing points...")
    all_points_x = []
    all_points_y = []
    all_sensor_ids = []
    frame_indices = [0] # Start index for each frame in the flat points array
    sensor_base_angles = np.array([0.0, 90.0, 180.0, 270.0], dtype=np.float32)
    if sensor_offsets_body_xy_m is None:
        sensor_offsets_body_xy_m = DEFAULT_SENSOR_OFFSETS_BODY_XY_M
    sensor_offsets_body_xy_m = np.asarray(sensor_offsets_body_xy_m, dtype=np.float32)
    if sensor_yaw_trims_deg is None:
        sensor_yaw_trims_deg = DEFAULT_SENSOR_YAW_TRIMS_DEG
    sensor_yaw_trims_deg = np.asarray(sensor_yaw_trims_deg, dtype=np.float32)
    if column_az_offsets_deg is None:
        column_az_offsets_deg = DEFAULT_COLUMN_AZ_OFFSETS_DEG
    column_az_offsets_deg = np.asarray(column_az_offsets_deg, dtype=np.float32)
    if yaw_corrections_deg is None:
        yaw_corrections_deg = np.zeros(len(records), dtype=np.float32)
    yaw_corrections_deg = np.asarray(yaw_corrections_deg, dtype=np.float32)
    if filter_profile is None:
        filter_profile = build_filter_profile(DEFAULT_FILTER_AGGRESSIVENESS)

    elev_corrections, elev_angles_rad, azimuth_offsets, row_indices, col_indices = build_projection_tables()
    
    count = 0 
    for rec_idx, rec in enumerate(records):
        if not frame_pose_is_usable(rec, filter_profile):
            frame_indices.append(len(all_points_x))
            continue

        # Global drone pose
        drone_x = rec['x']
        drone_y = rec['y']
        corrected_yaw_deg = rec['yaw'] + yaw_corrections_deg[rec_idx]
        drone_yaw_rad = math.radians(corrected_yaw_deg)
        if use_3d_ray_rotation:
            world_from_body = rotation_matrix_rz_ry_rx(
                drone_yaw_rad,
                rec['pitch'],
                rec['roll'],
            )
        else:
            world_from_body = rotation_matrix_rz_ry_rx(drone_yaw_rad, 0.0, 0.0)
        
        measurements = extract_frame_measurements(
            rec,
            row_indices,
            col_indices,
            filter_profile['map_min_range_m'],
            filter_profile['map_max_range_m'],
            row_filter_indices=row_filter_indices,
        )
        if measurements is None:
            frame_indices.append(len(all_points_x))
            continue

        dists_mm, sensor_idx, zone_idx_in_sensor = measurements
        
        dists_m = dists_mm / 1000.0
        
        zone_col_idx = col_indices[zone_idx_in_sensor]
        corrected_azimuth_deg = (
            azimuth_offsets[zone_idx_in_sensor]
            + column_az_offsets_deg[sensor_idx, zone_col_idx]
        )

        body_azimuth_deg = (
            sensor_base_angles[sensor_idx]
            + sensor_yaw_trims_deg[sensor_idx]
            + corrected_azimuth_deg
        )
        body_azimuth_rad = np.radians(body_azimuth_deg)
        if use_3d_ray_rotation:
            body_elevation_rad = elev_angles_rad[zone_idx_in_sensor]
            ray_body = np.column_stack((
                np.cos(body_elevation_rad) * np.cos(body_azimuth_rad),
                np.cos(body_elevation_rad) * np.sin(body_azimuth_rad),
                np.sin(body_elevation_rad),
            ))
            sensor_forward_comp = np.cos(body_elevation_rad) * np.cos(
                np.radians(corrected_azimuth_deg)
            )
            sensor_forward_comp = np.clip(sensor_forward_comp, 1e-4, None)
            scale = dists_m / sensor_forward_comp
        else:
            planar_d = dists_m * elev_corrections[zone_idx_in_sensor]
            ray_body = np.column_stack((
                planar_d * np.cos(body_azimuth_rad),
                planar_d * np.sin(body_azimuth_rad),
                np.zeros(len(sensor_idx), dtype=np.float32),
            ))
        sensor_offsets_body = np.column_stack((
            sensor_offsets_body_xy_m[sensor_idx, 0],
            sensor_offsets_body_xy_m[sensor_idx, 1],
            np.zeros(len(sensor_idx), dtype=np.float32),
        ))
        sensor_offsets_world = sensor_offsets_body @ world_from_body.T
        ray_world = ray_body @ world_from_body.T

        sensor_origins_x = drone_x + sensor_offsets_world[:, 0]
        sensor_origins_y = drone_y + sensor_offsets_world[:, 1]
        if use_3d_ray_rotation:
            px = sensor_origins_x + scale * ray_world[:, 0]
            py = sensor_origins_y + scale * ray_world[:, 1]
        else:
            px = sensor_origins_x + ray_world[:, 0]
            py = sensor_origins_y + ray_world[:, 1]

        px, py, sensor_idx = reject_isolated_points(
            px,
            py,
            sensor_idx,
            radius_m=OUTLIER_REJECT_RADIUS_M,
            min_neighbors=OUTLIER_REJECT_MIN_NEIGHBORS,
        )
        if len(px) == 0:
            frame_indices.append(len(all_points_x))
            continue

        all_points_x.extend(px)
        all_points_y.extend(py)
        all_sensor_ids.extend(sensor_idx)

        frame_indices.append(len(all_points_x))
        count += 1

    print(f"Generated {len(all_points_x)} points from {count} frames.")
    return (
        np.array(all_points_x, dtype=np.float32),
        np.array(all_points_y, dtype=np.float32),
        np.array(all_sensor_ids, dtype=np.uint8),
        frame_indices,
        records,
    )


def downsample_points(x, y, sensor_ids, limit):
    if len(x) <= limit:
        return x, y, sensor_ids

    step = int(math.ceil(len(x) / float(limit)))
    return x[::step], y[::step], sensor_ids[::step]


def build_hit_density_raster(points_x, points_y, xlim, ylim, cell_size_m):
    """Bin measured ToF endpoints; this does not infer free or unknown space."""
    if not np.isfinite(cell_size_m) or cell_size_m <= 0.0:
        raise ValueError("cell_size_m must be finite and positive")
    if len(points_x) == 0:
        return None, [xlim[0], xlim[1], ylim[0], ylim[1]]

    x_edges = np.arange(xlim[0], xlim[1] + cell_size_m, cell_size_m, dtype=np.float32)
    y_edges = np.arange(ylim[0], ylim[1] + cell_size_m, cell_size_m, dtype=np.float32)
    if len(x_edges) < 2 or len(y_edges) < 2:
        return None, [xlim[0], xlim[1], ylim[0], ylim[1]]

    hist, _, _ = np.histogram2d(points_y, points_x, bins=[y_edges, x_edges])
    raster = np.ma.masked_less(hist.astype(np.float32), float(OCCUPANCY_MIN_HIT_THRESHOLD))
    extent = [x_edges[0], x_edges[-1], y_edges[0], y_edges[-1]]
    return raster, extent


def build_occupancy_raster(points_x, points_y, xlim, ylim, cell_size_m):
    """Compatibility alias; the returned image is endpoint hit density, not occupancy."""
    return build_hit_density_raster(points_x, points_y, xlim, ylim, cell_size_m)


def load_ground_truth(filepath):
    """Load ground truth walls from CSV: x1,y1,x2,y2 per line."""
    walls = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line_number, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            fields = line.split(',')
            if len(fields) != 4:
                raise ValueError(f"{filepath}:{line_number}: expected exactly x1,y1,x2,y2")
            wall = [float(value) for value in fields]
            if not all(math.isfinite(value) for value in wall):
                raise ValueError(f"{filepath}:{line_number}: ground-truth coordinates must be finite")
            walls.append(wall)
    return walls if walls else None


def presentation_filter_hit_density(raster):
    """Presentation-only morphology for a ToF endpoint-density raster.

    Removes isolated cells and tiny clusters, applies closing to bridge
    short gaps, and optionally dilates thin walls.  Uses square/cross
    structuring elements to preserve corners.
    """
    if raster is None:
        return None
    if ndi is None:
        print("Warning: scipy not available, skipping morphological cleanup.")
        return raster

    occupied = ~raster.mask

    # 1. Remove small connected components
    labeled, num_features = ndi.label(occupied)
    if num_features > 0:
        component_sizes = ndi.sum(occupied, labeled, range(1, num_features + 1))
        small_mask = np.zeros_like(occupied)
        for i, size in enumerate(component_sizes):
            if size < POSTER_MIN_CLUSTER_CELLS:
                small_mask |= (labeled == (i + 1))
        occupied = occupied & ~small_mask

    # 2. Morphological closing (square kernel preserves corners)
    if POSTER_MORPH_CLOSING_SIZE > 1:
        struct_close = np.ones(
            (POSTER_MORPH_CLOSING_SIZE, POSTER_MORPH_CLOSING_SIZE), dtype=bool,
        )
        occupied = ndi.binary_closing(occupied, structure=struct_close)

    # 3. Optional light dilation (cross kernel keeps corners sharp)
    if POSTER_MORPH_DILATE and POSTER_MORPH_DILATE_SIZE > 1:
        struct_dilate = ndi.generate_binary_structure(2, 1)
        occupied = ndi.binary_dilation(occupied, structure=struct_dilate)

    # Fill newly-occupied cells with threshold value so they render
    cleaned_data = raster.data.copy()
    newly_occupied = occupied & raster.mask
    cleaned_data[newly_occupied] = float(OCCUPANCY_MIN_HIT_THRESHOLD)
    return np.ma.array(cleaned_data, mask=~occupied)


def clean_occupancy_raster(raster):
    """Compatibility alias for presentation_filter_hit_density()."""
    return presentation_filter_hit_density(raster)


def windows_explorer_path(path):
    """Return a Windows-friendly path string when running under WSL."""
    norm_path = os.path.abspath(path)
    if norm_path.startswith('/mnt/') and len(norm_path) > 6 and norm_path[5].isalpha():
        drive = norm_path[5].upper()
        suffix = norm_path[6:].replace('/', '\\')
        return f"{drive}:{suffix}"

    distro_name = os.environ.get('WSL_DISTRO_NAME')
    if distro_name:
        unc_path = norm_path.replace('/', '\\')
        return f"\\\\wsl.localhost\\{distro_name}{unc_path}"

    return norm_path


def choose_export_directory(default_dir):
    """Open a folder picker for export destination. Returns None on cancel."""
    if tk is None:
        print("tkinter unavailable, exporting to current directory instead.")
        return default_dir

    root = None
    try:
        root = tk.Tk()
        root.withdraw()
        selected_dir = filedialog.askdirectory(
            title="Choose Export Folder",
            initialdir=default_dir,
            mustexist=True,
        )
        return selected_dir or None
    except Exception as e:
        print(f"Error opening export folder dialog: {e}")
        return default_dir
    finally:
        if root is not None:
            root.destroy()


def sha256_file(filepath):
    digest = hashlib.sha256()
    with open(filepath, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def export_poster_images(
    points_x,
    points_y,
    cell_size_m,
    output_dir,
    ground_truth_file=None,
    ground_truth_rotation_deg=0.0,
    source_log_path=None,
    export_metadata=None,
):
    """Export traceable ToF endpoint-density images into a unique run directory."""
    if len(points_x) == 0:
        print("No points to export.")
        return None

    source_stem = Path(source_log_path).stem if source_log_path else 'unknown_log'
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    run_dir = Path(output_dir) / f"plot_scan_{source_stem}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    margin = POSTER_CROP_MARGIN_M
    xlim = (float(np.min(points_x)) - margin, float(np.max(points_x)) + margin)
    ylim = (float(np.min(points_y)) - margin, float(np.max(points_y)) + margin)

    raw_raster, extent = build_hit_density_raster(points_x, points_y, xlim, ylim, cell_size_m)
    if raw_raster is None:
        print("No hit-density data to export.")
        return None

    cleaned_raster = presentation_filter_hit_density(raw_raster)

    gt_walls = None
    if ground_truth_file:
        gt_walls = load_ground_truth(ground_truth_file)

    cmap = plt.cm.Greys.copy()
    cmap.set_bad(alpha=0.0)

    outputs = []
    for label, raster in [('raw', raw_raster), ('presentation_filtered', cleaned_raster)]:
        fig_e, ax_e = plt.subplots(figsize=(10, 10))
        ax_e.imshow(
            raster,
            extent=extent,
            origin='lower',
            cmap=cmap,
            interpolation='nearest',
            alpha=1.0,
            vmin=float(OCCUPANCY_MIN_HIT_THRESHOLD),
            vmax=OCCUPANCY_SATURATION_HITS,
        )
        if gt_walls is not None:
            for wall in gt_walls:
                gx, gy = rotate_all_points(
                    np.array([wall[0], wall[2]], dtype=np.float32),
                    np.array([wall[1], wall[3]], dtype=np.float32),
                    ground_truth_rotation_deg,
                )
                ax_e.plot(
                    gx, gy,
                    color='tab:red', linewidth=0.8, alpha=0.7,
                )
        ax_e.set_aspect('equal')
        ax_e.set_xlim(xlim)
        ax_e.set_ylim(ylim)
        ax_e.set_xlabel('X (m)')
        ax_e.set_ylabel('Y (m)')
        ax_e.grid(False)
        fig_e.tight_layout()
        fname = run_dir / f'tof_hit_density_{label}.png'
        fig_e.savefig(fname, dpi=300, bbox_inches='tight')
        print(f"Saved {fname}")
        fname_str = str(fname)
        explorer_fname = windows_explorer_path(fname_str)
        if explorer_fname != fname_str:
            print(f"Windows path: {explorer_fname}")
        plt.close(fig_e)
        outputs.append({
            'path': fname.name,
            'sha256': sha256_file(fname),
        })

    manifest = {
        'format_version': 1,
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'generator': str(Path(__file__).resolve()),
        'generator_sha256': sha256_file(Path(__file__).resolve()),
        'source_log': str(Path(source_log_path).resolve()) if source_log_path else None,
        'source_log_sha256': sha256_file(source_log_path) if source_log_path else None,
        'ground_truth': str(Path(ground_truth_file).resolve()) if ground_truth_file else None,
        'ground_truth_sha256': sha256_file(ground_truth_file) if ground_truth_file else None,
        'ground_truth_rotation_deg': float(ground_truth_rotation_deg),
        'interpretation': (
            'ToF endpoint hit density. Transparent cells are below the hit threshold; '
            'they are not proven free space. The presentation_filtered image applies morphology.'
        ),
        'point_count': int(len(points_x)),
        'cell_size_m': float(cell_size_m),
        'minimum_hits': int(OCCUPANCY_MIN_HIT_THRESHOLD),
        'saturation_hits': float(OCCUPANCY_SATURATION_HITS),
        'bounds_m': {'x': list(xlim), 'y': list(ylim)},
        'configuration': export_metadata or {},
        'outputs': outputs,
    }
    manifest_path = run_dir / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(f"Saved provenance manifest {manifest_path}")
    return run_dir


def build_trajectory_arrays(records, filter_profile):
    traj_x = np.array([
        r['x'] if frame_pose_is_usable(r, filter_profile) else np.nan for r in records
    ], dtype=np.float32)
    traj_y = np.array([
        r['y'] if frame_pose_is_usable(r, filter_profile) else np.nan for r in records
    ], dtype=np.float32)
    return traj_x, traj_y


def build_human_marker_arrays(records, filter_profile):
    human_x = []
    human_y = []
    human_frames = []

    for frame_idx, rec in enumerate(records):
        if (rec['pose_flags'] & POSEF_HUMAN_SEEN) == 0:
            continue
        if not frame_pose_is_usable(rec, filter_profile):
            continue
        if not np.isfinite(rec['x']) or not np.isfinite(rec['y']):
            continue

        human_x.append(rec['x'])
        human_y.append(rec['y'])
        human_frames.append(frame_idx)

    return (
        np.asarray(human_x, dtype=np.float32),
        np.asarray(human_y, dtype=np.float32),
        np.asarray(human_frames, dtype=np.int32),
    )


def compute_session_summary(records, filter_profile):
    total_records = len(records)
    if total_records == 0:
        return {
            'duration_s': 0.0,
            'armed_s': 0.0,
            'avg_of_q': 0.0,
            'avg_speed_mps': 0.0,
            'valid_count': 0,
            'valid_pct': 0.0,
            'session_count': 0,
            'timestamp_resets': 0,
            'rejection_counts': {},
        }

    host_ms = np.array([r['t_ms'] for r in records], dtype=np.float64)
    if total_records > 1:
        dts = np.diff(host_ms) * 0.001
        session_ids = np.array([r.get('session_id', 0) for r in records], dtype=np.int64)
        same_session = session_ids[1:] == session_ids[:-1]
        valid_dts = dts[(dts > 0) & same_session]
        duration_s = float(np.sum(valid_dts))
    else:
        dts = np.array([], dtype=np.float64)
        same_session = np.array([], dtype=bool)
        duration_s = 0.0

    armed_s = 0.0
    if total_records > 1:
        fc_armed = np.array([r['fc_armed'] != 0 for r in records], dtype=bool)
        for idx, dt in enumerate(dts):
            if dt > 0 and same_session[idx] and fc_armed[idx] and fc_armed[idx + 1]:
                armed_s += float(dt)

    fresh_of_mask = np.array([
        ((r['pose_flags'] & POSEF_OF_FRESH) != 0) and r['of_q'] > 0 for r in records
    ], dtype=bool)
    if np.any(fresh_of_mask):
        avg_of_q = float(np.mean([records[i]['of_q'] for i in np.where(fresh_of_mask)[0]]))
    else:
        avg_of_q = 0.0

    valid_mask = np.array([frame_pose_is_usable(r, filter_profile) for r in records], dtype=bool)
    valid_count = int(np.count_nonzero(valid_mask))
    valid_pct = (100.0 * valid_count / total_records) if total_records else 0.0
    rejection_counts = frame_rejection_counts(records, filter_profile)
    session_count = 1 + max(int(r.get('session_id', 0)) for r in records)

    if np.any(valid_mask):
        valid_indices = np.where(valid_mask)[0]
        avg_speed_mps = float(np.mean([
            math.hypot(records[i]['vx'], records[i]['vy']) for i in valid_indices
        ]))
    else:
        avg_speed_mps = 0.0

    return {
        'duration_s': duration_s,
        'armed_s': armed_s,
        'avg_of_q': avg_of_q,
        'avg_speed_mps': avg_speed_mps,
        'valid_count': valid_count,
        'valid_pct': valid_pct,
        'session_count': session_count,
        'timestamp_resets': max(0, session_count - 1),
        'rejection_counts': rejection_counts,
    }


def frame_points_slice(frame_indices, frame_idx):
    start_pt_idx = frame_indices[frame_idx]
    end_pt_idx = frame_indices[frame_idx + 1]
    return start_pt_idx, end_pt_idx


def rebuild_points(
    records,
    column_offsets_deg,
    use_manhattan_yaw,
    use_3d_ray_rotation,
    filter_profile,
    row_filter_indices=None,
):
    if use_manhattan_yaw:
        yaw_corrections_deg = estimate_manhattan_yaw_corrections(
            records,
            DEFAULT_SENSOR_OFFSETS_BODY_XY_M,
            DEFAULT_SENSOR_YAW_TRIMS_DEG,
            column_offsets_deg,
            filter_profile,
            row_filter_indices=row_filter_indices,
        )
    else:
        yaw_corrections_deg = np.zeros(len(records), dtype=np.float32)
    return compute_points(
        records,
        sensor_offsets_body_xy_m=DEFAULT_SENSOR_OFFSETS_BODY_XY_M,
        sensor_yaw_trims_deg=DEFAULT_SENSOR_YAW_TRIMS_DEG,
        column_az_offsets_deg=column_offsets_deg,
        yaw_corrections_deg=yaw_corrections_deg,
        use_3d_ray_rotation=use_3d_ray_rotation,
        filter_profile=filter_profile,
        row_filter_indices=row_filter_indices,
    )

# -----------------------------------------------------------------------------
# Visualization
# -----------------------------------------------------------------------------
def plot_data(
    points_x,
    points_y,
    point_sensor_ids,
    frame_indices,
    records,
    single_frame_only=False,
    use_startup_filter=True,
    ground_truth_file=None,
    room_size=None,
    room_offset=(0.0, 0.0),
    room_yaw_deg=0.0,
    source_log_path=None,
):
    if len(records) == 0:
        print("No records to plot.")
        return

    filter_profile = build_filter_profile(DEFAULT_FILTER_AGGRESSIVENESS, use_startup_filter=use_startup_filter)
    if use_startup_filter:
        configure_startup_filter(filter_profile, records)
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.45)
    traj_x_all, traj_y_all = build_trajectory_arrays(records, filter_profile)
    human_x_all, human_y_all, human_frames_all = build_human_marker_arrays(records, filter_profile)
    session_summary = compute_session_summary(records, filter_profile)
    view_state = {
        'single_frame_only': single_frame_only,
        'use_manhattan_yaw': False,
        'use_3d_ray_rotation': True,
        'filter_aggressiveness': DEFAULT_FILTER_AGGRESSIVENESS,
        'render_mode': 'points',
        'occupancy_cell_size_m': OCCUPANCY_CELL_SIZE_M,
        'row_mode': DEFAULT_ROW_MODE,
        'use_startup_filter': use_startup_filter,
        'poster_mode': False,
        'show_summary': True,
        'show_trajectory': True,
        'show_legend': True,
        'align_to_axis': False,
        'alignment_angle_deg': 0.0,
        'manual_rotation_deg': 0.0,
        'show_drone': True,
        'color_by_sensor': True,
        'show_humans': True,
    }
    point_state = {
        'x': points_x,
        'y': points_y,
        'sensor_ids': point_sensor_ids,
        'frame_indices': frame_indices,
        'traj_x': traj_x_all,
        'traj_y': traj_y_all,
        'human_x': human_x_all,
        'human_y': human_y_all,
        'human_frames': human_frames_all,
        'human_total': int(len(human_frames_all)),
        'session_summary': session_summary,
        'filter_profile': filter_profile,
    }

    sensor_scatters = {}
    for sensor_idx, (sensor_name, sensor_color) in enumerate(zip(SENSOR_NAMES, SENSOR_COLORS)):
        if not ENABLED_SENSOR_MASK[sensor_idx]:
            continue
        sensor_scatters[sensor_idx] = ax.scatter(
            [], [], s=3, c=sensor_color, alpha=0.55, label=sensor_name
        )
    mono_scatter = ax.scatter([], [], s=3, c='black', alpha=0.55, label='_nolegend_')
    mono_scatter.set_visible(False)
    occupancy_cmap = plt.cm.Greys.copy()
    occupancy_cmap.set_bad(alpha=0.0)
    occupancy_img = ax.imshow(
        np.ma.masked_all((2, 2), dtype=np.float32),
        extent=[-1.0, 1.0, -1.0, 1.0],
        origin='lower',
        cmap=occupancy_cmap,
        interpolation='nearest',
        alpha=1.0,
        visible=False,
        zorder=1,
    )
    trajectory, = ax.plot([], [], 'r-', linewidth=1, label='Trajectory')
    human_scatter = ax.scatter(
        [], [], s=70, marker='*', c='tab:red',
        edgecolors='black', linewidths=0.4, alpha=0.95,
        label='Vehicle pose at person-detection flag', zorder=12,
    )
    drone_marker, = ax.plot([], [], 'ko', markersize=6, zorder=10)
    heading_arrow = ax.annotate(
        '', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='magenta', lw=2.5, mutation_scale=15),
        zorder=11,
    )

    # Ground truth remains in the same displayed frame as the scan points.
    ground_truth_walls = None
    ground_truth_lines = []
    if ground_truth_file:
        ground_truth_walls = load_ground_truth(ground_truth_file)
        if ground_truth_walls:
            for wall in ground_truth_walls:
                line, = ax.plot(
                    [wall[0], wall[2]], [wall[1], wall[3]],
                    color='tab:red', linewidth=0.8, alpha=0.7, zorder=5,
                )
                ground_truth_lines.append(line)

    def update_ground_truth_overlay(rotation_deg):
        if not ground_truth_walls:
            return
        for wall, line in zip(ground_truth_walls, ground_truth_lines):
            gx, gy = rotate_all_points(
                np.array([wall[0], wall[2]], dtype=np.float32),
                np.array([wall[1], wall[3]], dtype=np.float32),
                rotation_deg,
            )
            line.set_data(gx, gy)

    # Room boundary overlay (drawn as a closed line so rotation works)
    room_line = None
    room_box = None
    room_defaults = None
    if room_size is not None:
        rw, rl = room_size
        ox, oy = room_offset
        room_box = {
            'width': float(rw),
            'length': float(rl),
            'center_x': float(ox),
            'center_y': float(oy),
            'yaw_deg': float(room_yaw_deg),
        }
        room_defaults = dict(room_box)
        room_corners_xy = build_room_outline_xy(
            room_box['width'],
            room_box['length'],
            room_box['center_x'],
            room_box['center_y'],
            room_box['yaw_deg'],
        )
        room_line, = ax.plot(
            room_corners_xy[:, 0], room_corners_xy[:, 1],
            color='tab:cyan', linewidth=2.0, linestyle='--',
            alpha=0.85, zorder=6, label='Room',
            scalex=False, scaley=False,
        )
    view_state['show_room'] = room_line is not None

    def build_room_outline_for_display():
        if room_box is None:
            return None
        return build_room_outline_xy(
            room_box['width'],
            room_box['length'],
            room_box['center_x'],
            room_box['center_y'],
            room_box['yaw_deg'],
        )

    def update_room_overlay(rotation_deg):
        if room_line is None or room_box is None:
            return
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        room_corners_xy = build_room_outline_for_display()
        if room_corners_xy is None:
            return
        if rotation_deg != 0.0:
            rx, ry = rotate_all_points(
                room_corners_xy[:, 0],
                room_corners_xy[:, 1],
                rotation_deg,
            )
            room_line.set_data(rx, ry)
        else:
            room_line.set_data(room_corners_xy[:, 0], room_corners_xy[:, 1])
        ax.set_xlim(xlim, auto=False)
        ax.set_ylim(ylim, auto=False)
        restore_main_axes_geometry()

    def print_room_overlay_state():
        if room_box is None:
            return
        print(f"Room overlay: {format_room_box_summary(room_box)}")
        print(f"Room CLI: {format_room_cli_args(room_box)}")

    ax.set_aspect('equal', adjustable='box', anchor='C')
    ax.grid(True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    main_axes_position = ax.get_position().frozen()
    legend = ax.legend()
    title = ax.set_title("")
    summary_text = ax.text(
        0.015,
        0.985,
        "",
        transform=ax.transAxes,
        va='top',
        ha='left',
        fontsize=9,
        bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': '0.7'},
    )
    ax_summary_toggle = plt.axes([0.02, 0.92, 0.085, 0.04])
    btn_summary_toggle = Button(ax_summary_toggle, 'Sum On')
    btn_summary_toggle.label.set_fontsize(8)
    ax_traj_toggle = plt.axes([0.11, 0.92, 0.09, 0.04])
    btn_traj_toggle = Button(ax_traj_toggle, 'Traj On')
    btn_traj_toggle.label.set_fontsize(8)
    ax_legend_toggle = plt.axes([0.205, 0.92, 0.095, 0.04])
    btn_legend_toggle = Button(ax_legend_toggle, 'Legend On')
    btn_legend_toggle.label.set_fontsize(8)
    ax_align = plt.axes([0.305, 0.92, 0.095, 0.04])
    btn_align = Button(ax_align, 'Align: Off')
    btn_align.label.set_fontsize(8)
    ax_drone_toggle = plt.axes([0.405, 0.92, 0.095, 0.04])
    btn_drone_toggle = Button(ax_drone_toggle, 'Drone On')
    btn_drone_toggle.label.set_fontsize(8)
    ax_color_toggle = plt.axes([0.505, 0.92, 0.095, 0.04])
    btn_color_toggle = Button(ax_color_toggle, 'Color On')
    btn_color_toggle.label.set_fontsize(8)
    ax_room_toggle = plt.axes([0.605, 0.92, 0.095, 0.04])
    btn_room_toggle = Button(
        ax_room_toggle,
        'Room On' if view_state['show_room'] else 'Room Off',
    )
    btn_room_toggle.label.set_fontsize(8)
    ax_human_toggle = plt.axes([0.705, 0.92, 0.095, 0.04])
    btn_human_toggle = Button(ax_human_toggle, 'Human On')
    btn_human_toggle.label.set_fontsize(8)

    # Calculate bounds
    if len(points_x) > 0:
        finite = np.isfinite(points_x) & np.isfinite(points_y)
        full_x = np.asarray(points_x)[finite]
        full_y = np.asarray(points_y)[finite]
        min_x, max_x = float(np.min(full_x)), float(np.max(full_x))
        min_y, max_y = float(np.min(full_y)), float(np.max(full_y))
        margin = 0.5
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)
    else:
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
    view_state['normal_xlim'] = ax.get_xlim()
    view_state['normal_ylim'] = ax.get_ylim()
    ax.set_autoscale_on(False)

    def restore_main_axes_geometry():
        ax.set_position(main_axes_position)
        ax.set_aspect('equal', adjustable='box', anchor='C')
        ax.set_autoscale_on(False)

    def effective_rotation_deg():
        angle = view_state['manual_rotation_deg']
        if view_state['align_to_axis']:
            angle += view_state['alignment_angle_deg']
        return wrap_deg180(angle)

    def update_view_limits():
        if len(point_state['x']) == 0:
            return
        finite = np.isfinite(point_state['x']) & np.isfinite(point_state['y'])
        fx = point_state['x'][finite]
        fy = point_state['y'][finite]
        if len(fx) == 0:
            return
        angle_deg = effective_rotation_deg()
        if angle_deg != 0.0:
            fx, fy = rotate_all_points(fx, fy, angle_deg)
        margin = 0.5
        view_state['normal_xlim'] = (float(np.min(fx)) - margin, float(np.max(fx)) + margin)
        view_state['normal_ylim'] = (float(np.min(fy)) - margin, float(np.max(fy)) + margin)
        restore_main_axes_geometry()

    # Sliders and buttons
    axcolor = 'lightgoldenrodyellow'
    time_max = len(records) - 1

    # Row 7 (top slider row): Manual rotation
    ax_rotate = plt.axes([0.25, 0.35, 0.60, 0.03], facecolor=axcolor)
    s_rotate = Slider(
        ax_rotate,
        'Rotate (deg)',
        -180.0,
        180.0,
        valinit=0.0,
        valstep=1.0,
        dragging=True,
    )
    # Row 6: Cell size
    ax_cell = plt.axes([0.25, 0.30, 0.60, 0.03], facecolor=axcolor)
    s_cell = Slider(
        ax_cell,
        'Cell (m)',
        MIN_OCCUPANCY_CELL_SIZE_M,
        MAX_OCCUPANCY_CELL_SIZE_M,
        valinit=OCCUPANCY_CELL_SIZE_M,
        valstep=0.01,
        dragging=True,
    )
    # Row 5: Frame slider (single-frame mode)
    ax_time = plt.axes([0.25, 0.24, 0.60, 0.03], facecolor=axcolor)
    try:
        s_time = Slider(ax_time, 'Frame', 0, time_max, valinit=0, valstep=1, dragging=True)
    except TypeError:
        s_time = Slider(ax_time, 'Frame', 0, time_max, valinit=0, valstep=1)
    # Row 4: Accum Start / End range sliders
    ax_range_start = plt.axes([0.25, 0.19, 0.60, 0.03], facecolor=axcolor)
    try:
        s_range_start = Slider(ax_range_start, 'Start', 0, time_max, valinit=0, valstep=1, dragging=True)
    except TypeError:
        s_range_start = Slider(ax_range_start, 'Start', 0, time_max, valinit=0, valstep=1)
    ax_range_end = plt.axes([0.25, 0.14, 0.60, 0.03], facecolor=axcolor)
    try:
        s_range_end = Slider(ax_range_end, 'End', 0, time_max, valinit=time_max, valstep=1, dragging=True)
    except TypeError:
        s_range_end = Slider(ax_range_end, 'End', 0, time_max, valinit=time_max, valstep=1)
    # Row 3: buttons + filter slider
    ax_toggle = plt.axes([0.02, 0.22, 0.16, 0.05])
    btn_toggle = Button(ax_toggle, 'View: Frame' if single_frame_only else 'View: Accum')
    ax_poster = plt.axes([0.02, 0.10, 0.16, 0.05])
    btn_poster = Button(ax_poster, 'Poster: Off')
    ax_rows = plt.axes([0.02, 0.03, 0.16, 0.05])
    btn_rows = Button(ax_rows, f'Rows: {row_mode_label(DEFAULT_ROW_MODE)}')
    ax_export = plt.axes([0.25, 0.10, 0.16, 0.05])
    btn_export = Button(ax_export, 'Export PNGs')
    ax_render = plt.axes([0.25, 0.03, 0.16, 0.05])
    btn_render = Button(ax_render, 'Render: Points')
    ax_startup = plt.axes([0.48, 0.03, 0.18, 0.05])
    btn_startup = Button(
        ax_startup,
        'Startup Filter: On' if use_startup_filter else 'Startup Filter: Off',
    )
    ax_filter = plt.axes([0.48, 0.10, 0.37, 0.03], facecolor=axcolor)
    ax_filter.axvline(DEFAULT_FILTER_AGGRESSIVENESS, color='red', linewidth=1.5, zorder=0)
    s_filter = Slider(
        ax_filter,
        'Filter',
        0.0,
        2.0,
        valinit=DEFAULT_FILTER_AGGRESSIVENESS,
        valstep=0.05,
        dragging=True,
    )
    
    def update(val):
        frame_idx = int(s_time.val)
        is_poster = view_state.get('poster_mode', False)
        rotation_deg = effective_rotation_deg()

        if view_state['single_frame_only']:
            start_pt_idx, end_pt_idx = frame_points_slice(point_state['frame_indices'], frame_idx)
        else:
            range_start, range_end = sorted((int(s_range_start.val), int(s_range_end.val)))
            start_pt_idx = point_state['frame_indices'][range_start]
            end_pt_idx = point_state['frame_indices'][min(range_end + 1, len(records))]

        current_x = point_state['x'][start_pt_idx:end_pt_idx]
        current_y = point_state['y'][start_pt_idx:end_pt_idx]
        current_sensor_ids = point_state['sensor_ids'][start_pt_idx:end_pt_idx]

        if rotation_deg != 0.0:
            current_x, current_y = rotate_all_points(
                current_x,
                current_y,
                rotation_deg,
            )

        # Update room overlay with current rotation
        if room_line is not None and view_state['show_room']:
            update_room_overlay(rotation_deg)
        update_ground_truth_overlay(rotation_deg)

        show_grid = view_state['render_mode'] == 'density' or is_poster

        if show_grid:
            if is_poster and len(current_x) > 0:
                p_margin = POSTER_CROP_MARGIN_M
                grid_xlim = (float(np.min(current_x)) - p_margin,
                             float(np.max(current_x)) + p_margin)
                grid_ylim = (float(np.min(current_y)) - p_margin,
                             float(np.max(current_y)) + p_margin)
            else:
                grid_xlim = ax.get_xlim()
                grid_ylim = ax.get_ylim()
            raster, extent = build_hit_density_raster(
                current_x,
                current_y,
                grid_xlim,
                grid_ylim,
                view_state['occupancy_cell_size_m'],
            )
            if is_poster and raster is not None:
                raster = presentation_filter_hit_density(raster)
            if raster is None:
                occupancy_img.set_data(np.ma.masked_all((2, 2), dtype=np.float32))
            else:
                occupancy_img.set_data(raster)
                occupancy_img.set_extent(extent)
                occupancy_img.set_clim(vmin=float(OCCUPANCY_MIN_HIT_THRESHOLD), vmax=OCCUPANCY_SATURATION_HITS)
            occupancy_img.set_visible(True)
        else:
            occupancy_img.set_visible(False)

        disp_x, disp_y, disp_sensor_ids = downsample_points(
            current_x,
            current_y,
            current_sensor_ids,
            MAX_DISPLAY_POINTS,
        )

        use_color = view_state['color_by_sensor']
        for sensor_idx, scatter in sensor_scatters.items():
            if show_grid or not use_color:
                scatter.set_offsets(np.empty((0, 2)))
                continue
            if len(disp_x) == 0:
                scatter.set_offsets(np.empty((0, 2)))
                continue

            sensor_mask = disp_sensor_ids == sensor_idx
            if np.any(sensor_mask):
                scatter.set_offsets(np.c_[disp_x[sensor_mask], disp_y[sensor_mask]])
            else:
                scatter.set_offsets(np.empty((0, 2)))

        if not use_color and not show_grid and len(disp_x) > 0:
            mono_scatter.set_offsets(np.c_[disp_x, disp_y])
            mono_scatter.set_visible(True)
        else:
            mono_scatter.set_offsets(np.empty((0, 2)))
            mono_scatter.set_visible(False)

        # Update Trajectory
        if view_state['single_frame_only']:
            traj_start = 0
            traj_end = frame_idx + 1
        else:
            traj_start, inclusive_end = sorted((int(s_range_start.val), int(s_range_end.val)))
            traj_end = inclusive_end + 1
        traj_x = point_state['traj_x'][traj_start:traj_end]
        traj_y = point_state['traj_y'][traj_start:traj_end]
        traj_disp_x, traj_disp_y, _ = downsample_points(
            traj_x,
            traj_y,
            np.zeros(len(traj_x), dtype=np.uint8),
            MAX_TRAJECTORY_POINTS,
        )
        if rotation_deg != 0.0:
            traj_disp_x, traj_disp_y = rotate_all_points(
                traj_disp_x,
                traj_disp_y,
                rotation_deg,
            )
        trajectory.set_data(traj_disp_x, traj_disp_y)

        if view_state['single_frame_only']:
            human_mask = point_state['human_frames'] == frame_idx
        else:
            human_start, human_end = sorted((int(s_range_start.val), int(s_range_end.val)))
            human_mask = (
                (point_state['human_frames'] >= human_start)
                & (point_state['human_frames'] <= human_end)
            )
        human_x = point_state['human_x'][human_mask]
        human_y = point_state['human_y'][human_mask]
        if rotation_deg != 0.0 and len(human_x) > 0:
            human_x, human_y = rotate_all_points(human_x, human_y, rotation_deg)
        if len(human_x) > 0:
            human_scatter.set_offsets(np.c_[human_x, human_y])
        else:
            human_scatter.set_offsets(np.empty((0, 2)))
        human_visible_count = int(len(human_x))

        # Update Drone Pos + heading arrow
        if view_state['single_frame_only']:
            drec = records[frame_idx]
        else:
            drec = records[max(int(s_range_start.val), int(s_range_end.val))]
        drone_pose_valid = frame_pose_is_usable(drec, point_state['filter_profile'])
        if drone_pose_valid:
            dx, dy = drec['x'], drec['y']
            yaw_rad = math.radians(drec['yaw'])
            if rotation_deg != 0.0:
                a_rad = math.radians(rotation_deg)
                c, s = math.cos(a_rad), math.sin(a_rad)
                dx, dy = dx * c - dy * s, dx * s + dy * c
                yaw_rad += a_rad
            drone_marker.set_data([dx], [dy])
            heading_arrow.xy = (dx + 0.6 * math.cos(yaw_rad), dy + 0.6 * math.sin(yaw_rad))
            heading_arrow.set_position((dx, dy))
        else:
            drone_marker.set_data([], [])

        # Poster mode: hide debug clutter, crop tight
        trajectory.set_visible(view_state['show_trajectory'] and not is_poster)
        human_scatter.set_visible(view_state['show_humans'] and not is_poster)
        drone_marker.set_visible(view_state['show_drone'] and drone_pose_valid and not is_poster)
        heading_arrow.set_visible(view_state['show_drone'] and drone_pose_valid and not is_poster)
        summary_text.set_visible(view_state['show_summary'] and not is_poster)
        if legend is not None:
            legend.set_visible(view_state['show_legend'] and not is_poster)

        if is_poster:
            ax.grid(False)
            ax.set_xlabel('')
            ax.set_ylabel('')
            ax.tick_params(labelbottom=False, labelleft=False, bottom=False, left=False)
            if len(current_x) > 0:
                p_margin = POSTER_CROP_MARGIN_M
                ax.set_xlim(float(np.min(current_x)) - p_margin,
                            float(np.max(current_x)) + p_margin)
                ax.set_ylim(float(np.min(current_y)) - p_margin,
                            float(np.max(current_y)) + p_margin)
            restore_main_axes_geometry()
            title.set_text('')
        else:
            ax.grid(True)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.tick_params(labelbottom=True, labelleft=True, bottom=True, left=True)
            if 'normal_xlim' in view_state:
                ax.set_xlim(view_state['normal_xlim'])
                ax.set_ylim(view_state['normal_ylim'])
            restore_main_axes_geometry()
            if view_state['single_frame_only']:
                mode_label = f"Frame {frame_idx + 1}/{len(records)}"
            else:
                display_start, display_end = sorted((int(s_range_start.val), int(s_range_end.val)))
                mode_label = f"Accum [{display_start}\u2013{display_end}]"
            title.set_text(
                f"{mode_label} | "
                f"Rows {row_mode_label(view_state['row_mode'])} | "
                f"Startup {'On' if view_state['use_startup_filter'] else 'Off'} | "
                f"Filter {view_state['filter_aggressiveness']:.2f} | "
                f"Rot {rotation_deg:+.0f}deg | "
                f"Cell {view_state['occupancy_cell_size_m']:.2f}m | "
                f"Valid {point_state['session_summary']['valid_count']}/{len(records)} | "
                f"Pts {len(disp_x):,}/{len(current_x):,} | "
                f"Human {human_visible_count}"
            )
            summary_text.set_text(
                "Session Summary\n"
                f"Flight: {point_state['session_summary']['duration_s']:.1f}s\n"
                f"Armed: {point_state['session_summary']['armed_s']:.1f}s\n"
                f"Avg OF q: {point_state['session_summary']['avg_of_q']:.1f}\n"
                f"Valid: {point_state['session_summary']['valid_pct']:.1f}%\n"
                f"Avg speed: {point_state['session_summary']['avg_speed_mps']:.2f} m/s\n"
                f"Sessions: {point_state['session_summary']['session_count']} "
                f"(resets {point_state['session_summary']['timestamp_resets']})\n"
                f"Person flags: {point_state['human_total']} (shown {human_visible_count})"
                + (
                    f"\nRoom: {room_box['width']:.2f} x {room_box['length']:.2f} m"
                    f" @ {room_box['yaw_deg']:.1f} deg"
                    f"\nCtr: ({room_box['center_x']:.2f}, {room_box['center_y']:.2f})"
                    if room_box is not None else ""
                )
            )

        fig.canvas.draw_idle()

    def toggle_view(_event):
        view_state['single_frame_only'] = not view_state['single_frame_only']
        btn_toggle.label.set_text(
            'View: Frame' if view_state['single_frame_only'] else 'View: Accum'
        )
        update(s_time.val)

    def rebuild_projection():
        profile = build_filter_profile(
            view_state['filter_aggressiveness'],
            use_startup_filter=view_state['use_startup_filter'],
        )
        if view_state['use_startup_filter']:
            configure_startup_filter(profile, records)
        else:
            profile['stable_hover_t_ms'] = 0
        new_x, new_y, new_sensor_ids, new_frame_indices, _ = rebuild_points(
            records,
            DEFAULT_COLUMN_AZ_OFFSETS_DEG,
            use_manhattan_yaw=view_state['use_manhattan_yaw'],
            use_3d_ray_rotation=view_state['use_3d_ray_rotation'],
            filter_profile=profile,
            row_filter_indices=row_mode_to_indices(view_state['row_mode']),
        )
        point_state['x'] = new_x
        point_state['y'] = new_y
        point_state['sensor_ids'] = new_sensor_ids
        point_state['frame_indices'] = new_frame_indices
        point_state['traj_x'], point_state['traj_y'] = build_trajectory_arrays(records, profile)
        human_x, human_y, human_frames = build_human_marker_arrays(records, profile)
        point_state['human_x'] = human_x
        point_state['human_y'] = human_y
        point_state['human_frames'] = human_frames
        point_state['human_total'] = int(len(human_frames))
        point_state['session_summary'] = compute_session_summary(records, profile)
        point_state['filter_profile'] = profile
        if view_state['align_to_axis']:
            view_state['alignment_angle_deg'] = estimate_alignment_angle_deg(new_x, new_y)
        update_view_limits()
        update(s_time.val)

    def toggle_poster(_event):
        view_state['poster_mode'] = not view_state['poster_mode']
        btn_poster.label.set_text(
            'Poster: On' if view_state['poster_mode'] else 'Poster: Off'
        )
        update(s_time.val)

    def toggle_summary(_event):
        view_state['show_summary'] = not view_state['show_summary']
        btn_summary_toggle.label.set_text(
            'Sum On' if view_state['show_summary'] else 'Sum Off'
        )
        update(s_time.val)

    def toggle_trajectory(_event):
        view_state['show_trajectory'] = not view_state['show_trajectory']
        btn_traj_toggle.label.set_text(
            'Traj On' if view_state['show_trajectory'] else 'Traj Off'
        )
        update(s_time.val)

    def toggle_legend(_event):
        view_state['show_legend'] = not view_state['show_legend']
        btn_legend_toggle.label.set_text(
            'Legend On' if view_state['show_legend'] else 'Legend Off'
        )
        update(s_time.val)

    def toggle_align(_event):
        view_state['align_to_axis'] = not view_state['align_to_axis']
        if view_state['align_to_axis']:
            angle = estimate_alignment_angle_deg(point_state['x'], point_state['y'])
            view_state['alignment_angle_deg'] = angle
            print(f"Axis alignment: {angle:.1f} deg")
        btn_align.label.set_text(
            'Align: On' if view_state['align_to_axis'] else 'Align: Off'
        )
        update_view_limits()
        update(s_time.val)

    def toggle_drone(_event):
        view_state['show_drone'] = not view_state['show_drone']
        btn_drone_toggle.label.set_text(
            'Drone On' if view_state['show_drone'] else 'Drone Off'
        )
        update(s_time.val)

    def toggle_color(_event):
        view_state['color_by_sensor'] = not view_state['color_by_sensor']
        btn_color_toggle.label.set_text(
            'Color On' if view_state['color_by_sensor'] else 'Color Off'
        )
        update(s_time.val)

    def toggle_humans(_event):
        view_state['show_humans'] = not view_state['show_humans']
        btn_human_toggle.label.set_text(
            'Human On' if view_state['show_humans'] else 'Human Off'
        )
        update(s_time.val)

    def toggle_room(_event):
        if room_line is None:
            return
        view_state['show_room'] = not view_state['show_room']
        room_line.set_visible(view_state['show_room'])
        btn_room_toggle.label.set_text(
            'Room On' if view_state['show_room'] else 'Room Off'
        )
        fig.canvas.draw_idle()

    def do_export_poster(_event):
        range_start, range_end = sorted((int(s_range_start.val), int(s_range_end.val)))
        start_pt_idx = point_state['frame_indices'][range_start]
        end_pt_idx = point_state['frame_indices'][min(range_end + 1, len(records))]
        export_x = point_state['x'][start_pt_idx:end_pt_idx]
        export_y = point_state['y'][start_pt_idx:end_pt_idx]
        export_rotation_deg = effective_rotation_deg()
        if export_rotation_deg != 0.0:
            export_x, export_y = rotate_all_points(export_x, export_y, export_rotation_deg)
        default_export_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = choose_export_directory(default_export_dir)
        if not output_dir:
            print("Export canceled.")
            return
        export_poster_images(
            export_x, export_y,
            view_state['occupancy_cell_size_m'],
            output_dir,
            ground_truth_file=ground_truth_file,
            ground_truth_rotation_deg=export_rotation_deg,
            source_log_path=source_log_path,
            export_metadata={
                'frame_start': range_start,
                'frame_end_inclusive': range_end,
                'row_mode': view_state['row_mode'],
                'filter_profile': point_state['filter_profile'],
                'use_manhattan_yaw': bool(view_state['use_manhattan_yaw']),
                'use_3d_ray_rotation': bool(view_state['use_3d_ray_rotation']),
                'display_rotation_deg': float(export_rotation_deg),
                'sensor_offsets_body_xy_m': DEFAULT_SENSOR_OFFSETS_BODY_XY_M.tolist(),
                'sensor_yaw_trims_deg': DEFAULT_SENSOR_YAW_TRIMS_DEG.tolist(),
                'column_azimuth_offsets_deg': DEFAULT_COLUMN_AZ_OFFSETS_DEG.tolist(),
                'sensor_geometry_status': SENSOR_GEOMETRY_STATUS,
            },
        )

    def toggle_render_mode(_event):
        view_state['render_mode'] = 'density' if view_state['render_mode'] == 'points' else 'points'
        btn_render.label.set_text(
            'Render: Hit Density' if view_state['render_mode'] == 'density' else 'Render: Points'
        )
        update(s_time.val)

    def toggle_row_mode(_event):
        view_state['row_mode'] = next_row_mode(view_state['row_mode'])
        btn_rows.label.set_text(f'Rows: {row_mode_label(view_state["row_mode"])}')
        rebuild_projection()

    def toggle_startup_filter(_event):
        view_state['use_startup_filter'] = not view_state['use_startup_filter']
        btn_startup.label.set_text(
            'Startup Filter: On' if view_state['use_startup_filter'] else 'Startup Filter: Off'
        )
        rebuild_projection()

    def update_filter_aggressiveness(_val):
        view_state['filter_aggressiveness'] = float(s_filter.val)
        rebuild_projection()

    def update_cell_size(_val):
        view_state['occupancy_cell_size_m'] = float(s_cell.val)
        update(s_time.val)

    def update_manual_rotation(_val):
        view_state['manual_rotation_deg'] = float(s_rotate.val)
        update_view_limits()
        update(s_time.val)

    def nudge_rotation(delta_deg):
        s_rotate.set_val(wrap_deg180(float(s_rotate.val) + delta_deg))

    def handle_key_press(event):
        if not event.key:
            return
        key, is_shift = parse_matplotlib_key(event.key)
        room_changed = False

        if room_box is not None:
            move_step = 0.05
            resize_step = 0.05
            rotate_step = 1.0

            if is_shift and key == 'j':
                room_box['width'] = max(0.05, room_box['width'] - resize_step)
                room_changed = True
            elif is_shift and key == 'l':
                room_box['width'] += resize_step
                room_changed = True
            elif is_shift and key == 'i':
                room_box['length'] += resize_step
                room_changed = True
            elif is_shift and key == 'k':
                room_box['length'] = max(0.05, room_box['length'] - resize_step)
                room_changed = True
            elif is_shift and key == 'r' and room_defaults is not None:
                room_box.update(room_defaults)
                room_changed = True
            elif key == 'j':
                room_box['center_x'] -= move_step
                room_changed = True
            elif key == 'l':
                room_box['center_x'] += move_step
                room_changed = True
            elif key == 'i':
                room_box['center_y'] += move_step
                room_changed = True
            elif key == 'k':
                room_box['center_y'] -= move_step
                room_changed = True
            elif key == 'u':
                room_box['yaw_deg'] = wrap_deg180(room_box['yaw_deg'] - rotate_step)
                room_changed = True
            elif key == 'o':
                room_box['yaw_deg'] = wrap_deg180(room_box['yaw_deg'] + rotate_step)
                room_changed = True
            elif key == 'p':
                print_room_overlay_state()
                return

        if room_changed:
            update(s_time.val)
            print_room_overlay_state()
            return

        if key in ('left', 'a'):
            nudge_rotation(-1.0)
        elif key in ('right', 'd'):
            nudge_rotation(1.0)
        elif is_shift and key == 'left':
            nudge_rotation(-5.0)
        elif is_shift and key == 'right':
            nudge_rotation(5.0)
        elif key in ('r', '0'):
            s_rotate.reset()

    s_rotate.on_changed(update_manual_rotation)
    s_cell.on_changed(update_cell_size)
    s_time.on_changed(update)
    s_range_start.on_changed(update)
    s_range_end.on_changed(update)
    btn_summary_toggle.on_clicked(toggle_summary)
    btn_traj_toggle.on_clicked(toggle_trajectory)
    btn_legend_toggle.on_clicked(toggle_legend)
    btn_align.on_clicked(toggle_align)
    btn_drone_toggle.on_clicked(toggle_drone)
    btn_color_toggle.on_clicked(toggle_color)
    btn_human_toggle.on_clicked(toggle_humans)
    btn_room_toggle.on_clicked(toggle_room)
    btn_toggle.on_clicked(toggle_view)
    btn_poster.on_clicked(toggle_poster)
    btn_rows.on_clicked(toggle_row_mode)
    btn_export.on_clicked(do_export_poster)
    btn_startup.on_clicked(toggle_startup_filter)
    btn_render.on_clicked(toggle_render_mode)
    s_filter.on_changed(update_filter_aggressiveness)
    fig.canvas.mpl_connect('key_press_event', handle_key_press)
    if room_box is not None:
        print(
            "Room controls: J/L move X, I/K move Y, U/O rotate, "
            "Shift+J/L width -, +, Shift+I/K length +, -, Shift+R reset, P print CLI."
        )
    update(0) # Init
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot ToF Scan Log")
    parser.add_argument("logfile", nargs='?', help="Path to .bin file")
    parser.add_argument(
        "--single-frame",
        action="store_true",
        help="Show only the current frame's points instead of the accumulated map",
    )
    parser.add_argument(
        "--ground-truth",
        default=None,
        help="CSV file with ground truth walls (x1,y1,x2,y2 per line)",
    )
    parser.add_argument(
        "--room",
        default=None,
        help="Room boundary as WxL in metres (e.g. '1.8x2.7'). Draws a rectangle overlay.",
    )
    parser.add_argument(
        "--room-offset",
        default=None,
        help="Room centre offset as X,Y in metres (e.g. '0.5,0.3'). Default: 0,0.",
    )
    parser.add_argument(
        "--room-yaw",
        type=float,
        default=0.0,
        help="Room box yaw in degrees for manual overlays.",
    )
    parser.add_argument(
        "--fit-room",
        action="store_true",
        help="Estimate an oriented best-fit room rectangle from the filtered scan points.",
    )
    parser.add_argument(
        "--allow-headerless-sclog3",
        action="store_true",
        help="Explicitly decode a separately verified headerless SCLOG3-layout file.",
    )
    args = parser.parse_args()

    room_size = None
    room_offset = (0.0, 0.0)
    room_yaw_deg = float(args.room_yaw)
    if args.room:
        parts = args.room.split('x')
        if len(parts) >= 2:
            room_size = (float(parts[0]), float(parts[1]))
    if args.room_offset:
        parts = args.room_offset.split(',')
        if len(parts) >= 2:
            room_offset = (float(parts[0]), float(parts[1]))

    filename = args.logfile

    # If no file provided, open file dialog
    if not filename and tk is not None:
        try:
            root = tk.Tk()
            root.withdraw()
            filename = filedialog.askopenfilename(
                title="Select Scan Log",
                filetypes=(("Binary Log", "*.bin"), ("All files", "*.*"))
            )
            root.destroy()
        except Exception as e:
            print(f"Error opening file dialog: {e}")

    if not filename:
        print("No file selected.")
        sys.exit(0)

    try:
        recs = parse_log(filename, allow_headerless=args.allow_headerless_sclog3)
        if not recs:
            print("No data found.")
            sys.exit(1)
        default_filter_profile = build_filter_profile(DEFAULT_FILTER_AGGRESSIVENESS)
        configure_startup_filter(default_filter_profile, recs)
        active_sensor_names = [
            SENSOR_NAMES[i] for i in range(NUM_SENSORS) if ENABLED_SENSOR_MASK[i]
        ]
        session_summary = compute_session_summary(recs, default_filter_profile)
        human_flag_count = sum(1 for r in recs if (r['pose_flags'] & POSEF_HUMAN_SEEN) != 0)
        print(
            f"Active sensors: {', '.join(active_sensor_names)} | "
            f"valid frames: {session_summary['valid_count']}/{len(recs)} | "
            f"valid={session_summary['valid_pct']:.1f}% | "
            f"flight={session_summary['duration_s']:.1f}s | "
            f"armed={session_summary['armed_s']:.1f}s | "
            f"avg_of_q={session_summary['avg_of_q']:.1f} | "
            f"avg_speed={session_summary['avg_speed_mps']:.2f}m/s | "
            f"human_flags={human_flag_count} | "
            f"sessions={session_summary['session_count']} | "
            f"timestamp_resets={session_summary['timestamp_resets']} | "
            f"filter={DEFAULT_FILTER_AGGRESSIVENESS:.2f}"
        )
        print(f"Sensor geometry: {SENSOR_GEOMETRY_STATUS}")
        rejected = {
            reason: count for reason, count in session_summary['rejection_counts'].items()
            if reason != 'accepted' and count
        }
        if rejected:
            print("Frame rejections: " + ", ".join(
                f"{reason}={count}" for reason, count in sorted(rejected.items())
            ))

        px, py, sensor_ids, idxs, recs = rebuild_points(
            recs,
            DEFAULT_COLUMN_AZ_OFFSETS_DEG,
            use_manhattan_yaw=False,
            use_3d_ray_rotation=True,
            filter_profile=default_filter_profile,
            row_filter_indices=row_mode_to_indices(DEFAULT_ROW_MODE),
        )
        if args.fit_room and room_size is None:
            room_fit = estimate_room_box(px, py)
            if room_fit is not None:
                room_size = (room_fit['width'], room_fit['length'])
                room_offset = (room_fit['center_x'], room_fit['center_y'])
                room_yaw_deg = room_fit['yaw_deg']
                print(f"Estimated room fit: {format_room_box_summary(room_fit)}")
            else:
                print("Estimated room fit unavailable: not enough valid scan points.")
        plot_data(
            px,
            py,
            sensor_ids,
            idxs,
            recs,
            single_frame_only=args.single_frame,
            ground_truth_file=args.ground_truth,
            room_size=room_size,
            room_offset=room_offset,
            room_yaw_deg=room_yaw_deg,
            source_log_path=filename,
        )

    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
        print("Usage: python plot_scan.py [path/to/scanlog.bin]")
        sys.exit(2)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(2)
