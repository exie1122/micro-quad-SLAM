import struct
import sys
import math
import argparse
import tkinter as tk
from tkinter import filedialog

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider
except ImportError as e:
    print("Error: This script requires 'numpy' and 'matplotlib'.")
    print("Please install them with: pip install numpy matplotlib")
    sys.exit(1)

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
DEFAULT_LOG_FILE = "scanlog.bin"
GRID_SIZE_M = 20.0  # Approx size of plot in meters
MAX_DISPLAY_POINTS = 200000
MAX_TRAJECTORY_POINTS = 20000
# VL53L5CX config
NUM_SENSORS = 4
ZONES_PER_SENSOR = 64 # 8x8
ROWS = 8
COLS = 8
FOV_H = 45.0 # Degrees
FOV_V = 45.0 # Degrees
MIDDLE_ROWS_ONLY = True
MIDDLE_ROW_INDICES = {3, 4}
# Logged sensor order is Front, Right, Back, Left.
SENSOR_NAMES = ["Front", "Right", "Back", "Left"]
SENSOR_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:purple"]
ENABLED_SENSOR_MASK = np.array([True, True, False, True], dtype=bool)
# Body-frame convention for sensor offsets:
#   +X = forward, +Y = right
# Fill these from measured frame geometry. Defaults are zero to preserve the
# previous center-origin behavior until calibrated values are entered.
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
MANHATTAN_MIN_SENSOR_POINTS = 4
MANHATTAN_MIN_CONTRIBUTING_SENSORS = 2
MANHATTAN_MIN_LINEARITY = 3.0
MANHATTAN_MAX_CORRECTION_DEG = 25.0
MANHATTAN_CORRECTION_GAIN = 0.45
MANHATTAN_MAX_STEP_DEG = 6.0
DEFAULT_FILTER_AGGRESSIVENESS = 1.0
MAP_MIN_RANGE_M = 0.15
MAP_MAX_RANGE_M = 3.50
MANHATTAN_MIN_RANGE_M = 0.20
MANHATTAN_MAX_RANGE_M = 2.50
MIN_OF_QUALITY = 40
MAX_ATT_AGE_MS = 400
MAX_LPOS_AGE_MS = 400
MAX_OF_AGE_MS = 400
MAX_RF_AGE_MS = 400
MAX_ABS_TILT_RAD = 0.40
POSEF_FC_LINK = 1 << 0
POSEF_FC_ARMED = 1 << 1
POSEF_ATT_FRESH = 1 << 2
POSEF_LPOS_FRESH = 1 << 3
POSEF_OF_FRESH = 1 << 4
POSEF_RF_FRESH = 1 << 5
POSEF_SYS_FRESH = 1 << 6
POSEF_ALT_VALID = 1 << 7
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

# -----------------------------------------------------------------------------
# Parsing
# -----------------------------------------------------------------------------
def parse_log(filename):
    print(f"Reading {filename}...")
    records = []
    
    with open(filename, 'rb') as f:
        # Check for header
        header = f.read(7)
        if header == b'SCLOG3\n':
            print("Found SCLOG3 header.")
        else:
            print("No header/unknown format, attempting to read from start.")
            f.seek(0)
            
        while True:
            chunk = f.read(RECORD_SIZE)
            if len(chunk) < RECORD_SIZE:
                break
                
            data = struct.unpack(SCAN_FMT, chunk)
            
            # Extract basic pose
            rec = {
                't_ms': data[1],
                'scan_ms': data[2],
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
                'of_q': data[32],
                'alt_src': data[33],
                'rf_src': data[34],
                'of_src': data[35],
                'fc_armed': data[36],
                'grid_bytes': data[41]
            }
            records.append(rec)
            
    print(f"Loaded {len(records)} records.")
    return records


def wrap_deg180(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def nearest_manhattan_axis_deg(angle_deg):
    return 90.0 * round(angle_deg / 90.0)


def build_filter_profile(aggressiveness):
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
        'max_abs_tilt_rad': max(0.18, 0.60 - 0.20 * a),
        'manhattan_min_linearity': 2.0 + 1.0 * a,
    }


def frame_pose_is_usable(rec, filter_profile):
    if not np.isfinite(rec['x']) or not np.isfinite(rec['y']) or not np.isfinite(rec['yaw']):
        return False
    if not np.isfinite(rec['roll']) or not np.isfinite(rec['pitch']):
        return False
    if (rec['pose_flags'] & REQUIRED_POSE_FLAGS) != REQUIRED_POSE_FLAGS:
        return False
    if rec['att_age_ms'] > filter_profile['max_att_age_ms']:
        return False
    if rec['lpos_age_ms'] > filter_profile['max_lpos_age_ms']:
        return False
    if rec['of_age_ms'] > filter_profile['max_of_age_ms']:
        return False
    if rec['of_q'] < filter_profile['min_of_quality']:
        return False
    if rec['alt_src'] in (2, 3) and rec['rf_age_ms'] > filter_profile['max_rf_age_ms']:
        return False
    if abs(rec['roll']) > filter_profile['max_abs_tilt_rad']:
        return False
    if abs(rec['pitch']) > filter_profile['max_abs_tilt_rad']:
        return False
    return True


def extract_frame_measurements(rec, row_indices, col_indices, min_range_m, max_range_m):
    raw_dist = np.frombuffer(rec['grid_bytes'], dtype=np.uint16)
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

    if MIDDLE_ROWS_ONLY:
        row_mask = np.isin(row_indices[zone_idx_in_sensor], list(MIDDLE_ROW_INDICES))
        if not np.any(row_mask):
            return None
        dists_mm = dists_mm[row_mask]
        sensor_idx = sensor_idx[row_mask]
        zone_idx_in_sensor = zone_idx_in_sensor[row_mask]

    return dists_mm, sensor_idx, zone_idx_in_sensor


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
):
    if not frame_pose_is_usable(rec, filter_profile):
        return None

    measurements = extract_frame_measurements(
        rec,
        row_indices,
        col_indices,
        filter_profile['manhattan_min_range_m'],
        filter_profile['manhattan_max_range_m'],
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
            px = sensor_origins_x + dists_m * ray_world[:, 0]
            py = sensor_origins_y + dists_m * ray_world[:, 1]
        else:
            px = sensor_origins_x + ray_world[:, 0]
            py = sensor_origins_y + ray_world[:, 1]
        
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


def build_trajectory_arrays(records, filter_profile):
    traj_x = np.array([
        r['x'] if frame_pose_is_usable(r, filter_profile) else np.nan for r in records
    ], dtype=np.float32)
    traj_y = np.array([
        r['y'] if frame_pose_is_usable(r, filter_profile) else np.nan for r in records
    ], dtype=np.float32)
    return traj_x, traj_y


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
        }

    host_ms = np.array([r['t_ms'] for r in records], dtype=np.float64)
    duration_s = max(0.0, (host_ms[-1] - host_ms[0]) * 0.001) if total_records > 1 else 0.0

    if total_records > 1:
        dts = np.diff(host_ms) * 0.001
        positive_dts = dts[dts > 0]
        median_dt = float(np.median(positive_dts)) if len(positive_dts) > 0 else 0.0
    else:
        dts = np.array([], dtype=np.float64)
        median_dt = 0.0

    armed_s = 0.0
    if total_records > 1:
        fc_armed = np.array([r['fc_armed'] != 0 for r in records], dtype=bool)
        for idx, dt in enumerate(dts):
            if fc_armed[idx] and fc_armed[idx + 1]:
                armed_s += float(dt)
        if fc_armed[-1]:
            armed_s += median_dt

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
    }


def frame_points_slice(frame_indices, frame_idx):
    start_pt_idx = frame_indices[frame_idx]
    end_pt_idx = frame_indices[frame_idx + 1]
    return start_pt_idx, end_pt_idx


def rebuild_points(records, column_offsets_deg, use_manhattan_yaw, use_3d_ray_rotation, filter_profile):
    if use_manhattan_yaw:
        yaw_corrections_deg = estimate_manhattan_yaw_corrections(
            records,
            DEFAULT_SENSOR_OFFSETS_BODY_XY_M,
            DEFAULT_SENSOR_YAW_TRIMS_DEG,
            column_offsets_deg,
            filter_profile,
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
    use_manhattan_yaw=False,
    use_3d_ray_rotation=True,
):
    if len(records) == 0:
        print("No records to plot.")
        return

    filter_profile = build_filter_profile(DEFAULT_FILTER_AGGRESSIVENESS)
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.34)
    traj_x_all, traj_y_all = build_trajectory_arrays(records, filter_profile)
    session_summary = compute_session_summary(records, filter_profile)
    view_state = {
        'single_frame_only': single_frame_only,
        'use_manhattan_yaw': use_manhattan_yaw,
        'use_3d_ray_rotation': use_3d_ray_rotation,
        'filter_aggressiveness': DEFAULT_FILTER_AGGRESSIVENESS,
    }
    point_state = {
        'x': points_x,
        'y': points_y,
        'sensor_ids': point_sensor_ids,
        'frame_indices': frame_indices,
        'traj_x': traj_x_all,
        'traj_y': traj_y_all,
        'session_summary': session_summary,
    }
    
    # Initial plot: all points shown initially? Or starts empty?
    # User asked for "gradual addition", maybe start with 10% or just 0.
    
    sensor_scatters = {}
    for sensor_idx, (sensor_name, sensor_color) in enumerate(zip(SENSOR_NAMES, SENSOR_COLORS)):
        if not ENABLED_SENSOR_MASK[sensor_idx]:
            continue
        sensor_scatters[sensor_idx] = ax.scatter(
            [], [], s=3, c=sensor_color, alpha=0.55, label=sensor_name
        )
    trajectory, = ax.plot([], [], 'r-', linewidth=1, label='Trajectory')
    drone_marker, = ax.plot([], [], 'ro', markersize=5)
    
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
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
    
    # Calculate bounds
    if len(points_x) > 0:
        full_x = [r['x'] for r in records]
        full_y = [r['y'] for r in records]
        
        # Use trajectory bounds to start, expanded slightly
        min_x, max_x = min(full_x), max(full_x)
        min_y, max_y = min(full_y), max(full_y)
        
        # Expand for points (points go out 4m)
        margin = 4.0
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)
    else:
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)

    # Slider
    axcolor = 'lightgoldenrodyellow'
    ax_time = plt.axes([0.25, 0.18, 0.60, 0.03], facecolor=axcolor)
    ax_toggle = plt.axes([0.02, 0.16, 0.16, 0.05])
    btn_toggle = Button(ax_toggle, 'View: Frame' if single_frame_only else 'View: Accum')
    ax_manhattan = plt.axes([0.02, 0.10, 0.16, 0.05])
    btn_manhattan = Button(
        ax_manhattan,
        'Manhattan: On' if use_manhattan_yaw else 'Manhattan: Off',
    )
    ax_ray = plt.axes([0.25, 0.10, 0.16, 0.05])
    btn_ray = Button(
        ax_ray,
        '3D Rays: On' if use_3d_ray_rotation else '3D Rays: Off',
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
    
    time_max = len(records) - 1
    try:
        s_time = Slider(ax_time, 'Frame', 0, time_max, valinit=0, valstep=1, dragging=False)
    except TypeError:
        s_time = Slider(ax_time, 'Frame', 0, time_max, valinit=0, valstep=1)
    
    def update(val):
        frame_idx = int(s_time.val)
        
        if view_state['single_frame_only']:
            start_pt_idx, end_pt_idx = frame_points_slice(point_state['frame_indices'], frame_idx)
        else:
            start_pt_idx = 0
            end_pt_idx = point_state['frame_indices'][frame_idx + 1]

        current_x = point_state['x'][start_pt_idx:end_pt_idx]
        current_y = point_state['y'][start_pt_idx:end_pt_idx]
        current_sensor_ids = point_state['sensor_ids'][start_pt_idx:end_pt_idx]
        disp_x, disp_y, disp_sensor_ids = downsample_points(
            current_x,
            current_y,
            current_sensor_ids,
            MAX_DISPLAY_POINTS,
        )
        
        for sensor_idx, scatter in sensor_scatters.items():
            if len(disp_x) == 0:
                scatter.set_offsets(np.empty((0, 2)))
                continue

            sensor_mask = disp_sensor_ids == sensor_idx
            if np.any(sensor_mask):
                scatter.set_offsets(np.c_[disp_x[sensor_mask], disp_y[sensor_mask]])
            else:
                scatter.set_offsets(np.empty((0, 2)))
            
        # Update Trajectory
        traj_x = point_state['traj_x'][:frame_idx + 1]
        traj_y = point_state['traj_y'][:frame_idx + 1]
        traj_disp_x, traj_disp_y, _ = downsample_points(
            traj_x,
            traj_y,
            np.zeros(len(traj_x), dtype=np.uint8),
            MAX_TRAJECTORY_POINTS,
        )
        trajectory.set_data(traj_disp_x, traj_disp_y)
        
        # Update Drone Pos
        drone_marker.set_data([records[frame_idx]['x']], [records[frame_idx]['y']])
        mode_name = "Single Frame" if view_state['single_frame_only'] else "Accumulated"
        title.set_text(
            f"{mode_name} | Manhattan {'On' if view_state['use_manhattan_yaw'] else 'Off'} | "
            f"3D Rays {'On' if view_state['use_3d_ray_rotation'] else 'Off'} | "
            f"Filter {view_state['filter_aggressiveness']:.2f} | "
            f"Valid Frames {point_state['session_summary']['valid_count']}/{len(records)} | "
            f"Frame {frame_idx + 1}/{len(records)} | "
            f"Points shown {len(disp_x):,}/{len(current_x):,}"
        )
        summary_text.set_text(
            "Session Summary\n"
            f"Flight: {point_state['session_summary']['duration_s']:.1f}s\n"
            f"Armed: {point_state['session_summary']['armed_s']:.1f}s\n"
            f"Avg OF q: {point_state['session_summary']['avg_of_q']:.1f}\n"
            f"Valid: {point_state['session_summary']['valid_pct']:.1f}%\n"
            f"Avg speed: {point_state['session_summary']['avg_speed_mps']:.2f} m/s"
        )
        
        fig.canvas.draw_idle()

    def toggle_view(_event):
        view_state['single_frame_only'] = not view_state['single_frame_only']
        btn_toggle.label.set_text(
            'View: Frame' if view_state['single_frame_only'] else 'View: Accum'
        )
        update(s_time.val)

    def rebuild_projection():
        profile = build_filter_profile(view_state['filter_aggressiveness'])
        new_x, new_y, new_sensor_ids, new_frame_indices, _ = rebuild_points(
            records,
            DEFAULT_COLUMN_AZ_OFFSETS_DEG,
            use_manhattan_yaw=view_state['use_manhattan_yaw'],
            use_3d_ray_rotation=view_state['use_3d_ray_rotation'],
            filter_profile=profile,
        )
        point_state['x'] = new_x
        point_state['y'] = new_y
        point_state['sensor_ids'] = new_sensor_ids
        point_state['frame_indices'] = new_frame_indices
        point_state['traj_x'], point_state['traj_y'] = build_trajectory_arrays(records, profile)
        point_state['session_summary'] = compute_session_summary(records, profile)
        update(s_time.val)

    def toggle_manhattan(_event):
        view_state['use_manhattan_yaw'] = not view_state['use_manhattan_yaw']
        btn_manhattan.label.set_text(
            'Manhattan: On' if view_state['use_manhattan_yaw'] else 'Manhattan: Off'
        )
        rebuild_projection()

    def toggle_ray_mode(_event):
        view_state['use_3d_ray_rotation'] = not view_state['use_3d_ray_rotation']
        btn_ray.label.set_text(
            '3D Rays: On' if view_state['use_3d_ray_rotation'] else '3D Rays: Off'
        )
        rebuild_projection()

    def update_filter_aggressiveness(_val):
        view_state['filter_aggressiveness'] = float(s_filter.val)
        rebuild_projection()

    s_time.on_changed(update)
    btn_toggle.on_clicked(toggle_view)
    btn_manhattan.on_clicked(toggle_manhattan)
    btn_ray.on_clicked(toggle_ray_mode)
    s_filter.on_changed(update_filter_aggressiveness)
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
        "--manhattan-yaw",
        action="store_true",
        help="Apply per-frame Manhattan yaw correction before projection",
    )
    args = parser.parse_args()

    filename = args.logfile

    # If no file provided, open file dialog
    if not filename:
        try:
            root = tk.Tk()
            root.withdraw() # Hide the main window
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
        recs = parse_log(filename)
        if not recs:
            print("No data found.")
            sys.exit(1)
        default_filter_profile = build_filter_profile(DEFAULT_FILTER_AGGRESSIVENESS)
        active_sensor_names = [
            SENSOR_NAMES[i] for i in range(NUM_SENSORS) if ENABLED_SENSOR_MASK[i]
        ]
        session_summary = compute_session_summary(recs, default_filter_profile)
        print(
            f"Active sensors: {', '.join(active_sensor_names)} | "
            f"valid frames: {session_summary['valid_count']}/{len(recs)} | "
            f"valid={session_summary['valid_pct']:.1f}% | "
            f"flight={session_summary['duration_s']:.1f}s | "
            f"armed={session_summary['armed_s']:.1f}s | "
            f"avg_of_q={session_summary['avg_of_q']:.1f} | "
            f"avg_speed={session_summary['avg_speed_mps']:.2f}m/s | "
            f"filter={DEFAULT_FILTER_AGGRESSIVENESS:.2f}"
        )
            
        px, py, sensor_ids, idxs, recs = rebuild_points(
            recs,
            DEFAULT_COLUMN_AZ_OFFSETS_DEG,
            use_manhattan_yaw=args.manhattan_yaw,
            use_3d_ray_rotation=True,
            filter_profile=default_filter_profile,
        )
        plot_data(
            px,
            py,
            sensor_ids,
            idxs,
            recs,
            single_frame_only=args.single_frame,
            use_manhattan_yaw=args.manhattan_yaw,
            use_3d_ray_rotation=True,
        )
        
    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
        print("Usage: python plot_scan.py [path/to/scanlog.bin]")
