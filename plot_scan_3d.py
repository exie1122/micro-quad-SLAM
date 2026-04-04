#!/usr/bin/env python3
"""3D point cloud viewer for drone ToF scan logs using the Ursina engine.

Reuses parsing, filtering, and projection logic from plot_scan.py but
retains the Z (altitude) component that the 2D viewer discards.

Usage:
    python plot_scan_3d.py [scanlog.bin]
    python plot_scan_3d.py [scanlog.bin] --start-frame 100 --end-frame 250 --export-json
    python plot_scan_3d.py [scanlog.bin] --export-json out/scan.json --no-viewer

Controls:
    Right-click + drag  : Orbit camera
    Middle-click + drag : Pan camera
    Scroll wheel        : Zoom
    1 / 2               : Decrease / increase point size
    C                   : Cycle colour mode (sensor / height / mono)
    T                   : Toggle trajectory
    G                   : Toggle ground plane
    R                   : Toggle room box
    Shift+R             : Reset room transform to CLI values
    J / L               : Move room in world X
    I / K               : Move room in world Y
    Y / N               : Move room up / down
    U / O               : Rotate room around vertical axis
    Shift+J / L         : Shrink / grow room width
    Shift+I / K         : Grow / shrink room length
    Shift+U / O         : Shrink / grow room height
    P                   : Save room preset for this log file
    E                   : Export current frame window to JSON
    H                   : Toggle screenshot mode (hide/show all GUI)
"""

import sys
import math
import json
import os
import argparse
import numpy as np

from plot_scan import (
    parse_log,
    build_filter_profile,
    compute_stable_hover_t_ms,
    frame_pose_is_usable,
    extract_frame_measurements,
    build_projection_tables,
    rotation_matrix_rz_ry_rx,
    estimate_manhattan_yaw_corrections,
    DEFAULT_SENSOR_OFFSETS_BODY_XY_M,
    DEFAULT_SENSOR_YAW_TRIMS_DEG,
    DEFAULT_COLUMN_AZ_OFFSETS_DEG,
    DEFAULT_FILTER_AGGRESSIVENESS,
    ENABLED_SENSOR_MASK,
    SENSOR_NAMES,
    NUM_SENSORS,
    OUTLIER_REJECT_RADIUS_M,
    OUTLIER_REJECT_MIN_NEIGHBORS,
    row_mode_to_indices,
)

# ── Sensor colours – saturated for visibility on white background ─────────────
SENSOR_RGB = [
    (0.05, 0.30, 0.70),   # Front  – strong blue
    (0.85, 0.35, 0.00),   # Right  – deep orange
    (0.00, 0.55, 0.15),   # Back   – forest green
    (0.55, 0.10, 0.70),   # Left   – vivid purple
]

COLOR_MODES = ['sensor', 'height', 'depth', 'frame', 'black']

MAX_DISPLAY_POINTS = 250_000

# Row modes for 3D viewer.
#
# VL53L5CX: 45° vertical FOV, 8 rows → 5.625° per row.
#   Rows 3,4 (Both): ±2.8° from horizontal → ±0.10 m at 2 m range.
#                     Clean wall data, minimal vertical spread.
#   All rows (0‑7):  ±19.7° → ±0.67 m at 2 m, ±1.18 m at 3.5 m.
#                     Captures floor / ceiling from extreme rows, combined
#                     with rangefinder‑derived altitude for true Z placement.
ROW_MODES = ['all', 'both']


def next_row_mode_3d(mode):
    return 'both' if mode == 'all' else 'all'


def row_mode_display(mode):
    """Short label for button text."""
    return 'All' if mode == 'all' else 'Mid'


def row_mode_detail(mode):
    """Longer description for HUD info line."""
    if mode == 'all':
        return 'All 8 rows ±19.7°'
    return 'Rows 3-4 ±2.8°'


def row_mode_filter(mode):
    """Return row_filter_indices for compute_points_3d (None = all rows)."""
    if mode == 'all':
        return None
    return row_mode_to_indices(mode)


def clamp_frame_range(start_frame, end_frame, num_records):
    """Clamp a start/end frame pair to the available record range."""
    if num_records <= 0:
        return 0, 0
    max_frame = num_records - 1
    start_frame = max(0, min(int(start_frame), max_frame))
    end_frame = max(0, min(int(end_frame), max_frame))
    if end_frame < start_frame:
        end_frame = start_frame
    return start_frame, end_frame


def prepare_scan_dataset(
    records,
    use_manhattan=False,
    use_startup_filter=True,
    row_mode='all',
    filter_aggressiveness=DEFAULT_FILTER_AGGRESSIVENESS,
):
    """Compute the filtered 3D dataset shared by the viewer and JSON export."""
    profile = build_filter_profile(
        filter_aggressiveness,
        use_startup_filter=use_startup_filter,
    )
    if use_startup_filter:
        profile['stable_hover_t_ms'] = compute_stable_hover_t_ms(records)
    else:
        profile['stable_hover_t_ms'] = 0

    row_filter_indices = row_mode_filter(row_mode)
    if use_manhattan:
        yaw_corrections_deg = estimate_manhattan_yaw_corrections(
            records,
            DEFAULT_SENSOR_OFFSETS_BODY_XY_M,
            DEFAULT_SENSOR_YAW_TRIMS_DEG,
            DEFAULT_COLUMN_AZ_OFFSETS_DEG,
            profile,
            row_filter_indices=row_filter_indices,
        )
    else:
        yaw_corrections_deg = np.zeros(len(records), dtype=np.float32)

    px, py, pz, sids, frame_indices = compute_points_3d(
        records,
        yaw_corrections_deg=yaw_corrections_deg,
        filter_profile=profile,
        row_filter_indices=row_filter_indices,
    )

    if len(px) > 0:
        cx = float(np.mean(px))
        cy = float(np.mean(py))
        cz = float(np.mean(pz))
    else:
        cx = cy = cz = 0.0

    traj_x, traj_y, traj_z = build_trajectory_3d(records, profile)

    return {
        'px': px,
        'py': py,
        'pz': pz,
        'sids': sids,
        'frame_indices': frame_indices,
        'cx': cx,
        'cy': cy,
        'cz': cz,
        'ux': px - cx,
        'uy': pz - cz,      # alt → viewer Y
        'uz': py - cy,
        'traj_x': traj_x,
        'traj_y': traj_y,
        'traj_z': traj_z,
        'filter_profile': profile,
    }


def build_point_frame_ids(frame_indices, start_frame, end_frame):
    """Per-point frame ids for the selected frame window."""
    start_pt = frame_indices[start_frame]
    end_pt = frame_indices[end_frame + 1]
    count = end_pt - start_pt
    if count <= 0:
        return np.empty(0, dtype=np.int32)

    point_frames = np.empty(count, dtype=np.int32)
    for frame_idx in range(start_frame, end_frame + 1):
        lo = frame_indices[frame_idx] - start_pt
        hi = frame_indices[frame_idx + 1] - start_pt
        if hi > lo:
            point_frames[lo:hi] = frame_idx
    return point_frames


def build_export_payload(
    logfile,
    dataset,
    records,
    start_frame,
    end_frame,
    row_mode,
    use_manhattan,
    use_startup_filter,
    filter_aggressiveness,
):
    """Create a JSON-serializable payload for the selected frame window."""
    num_records = len(records)
    start_frame, end_frame = clamp_frame_range(start_frame, end_frame, num_records)
    frame_indices = dataset['frame_indices']
    start_pt = frame_indices[start_frame]
    end_pt = frame_indices[end_frame + 1]
    point_frames = build_point_frame_ids(frame_indices, start_frame, end_frame)

    traj_slice = slice(start_frame, end_frame + 1)
    tx = dataset['traj_x'][traj_slice]
    ty = dataset['traj_y'][traj_slice]
    tz = dataset['traj_z'][traj_slice]
    traj_valid = np.isfinite(tx) & np.isfinite(ty) & np.isfinite(tz)
    traj_frames = np.arange(start_frame, end_frame + 1, dtype=np.int32)[traj_valid]

    return {
        'meta': {
            'generated_by': 'plot_scan_3d.py',
            'source_log': os.path.abspath(logfile),
            'frame_range': {
                'start': start_frame,
                'end': end_frame,
                'count': end_frame - start_frame + 1,
            },
            'point_count': int(end_pt - start_pt),
            'row_mode': row_mode,
            'use_manhattan': bool(use_manhattan),
            'use_startup_filter': bool(use_startup_filter),
            'filter_aggressiveness': float(filter_aggressiveness),
            'sensor_names': list(SENSOR_NAMES),
            'coordinate_frame': 'world_m',
        },
        'center_m': {
            'x': float(dataset['cx']),
            'y': float(dataset['cy']),
            'z': float(dataset['cz']),
        },
        'points': {
            'x': dataset['px'][start_pt:end_pt].astype(float).tolist(),
            'y': dataset['py'][start_pt:end_pt].astype(float).tolist(),
            'z': dataset['pz'][start_pt:end_pt].astype(float).tolist(),
            'sensor_id': dataset['sids'][start_pt:end_pt].astype(int).tolist(),
            'frame_index': point_frames.astype(int).tolist(),
        },
        'trajectory': {
            'frame_index': traj_frames.astype(int).tolist(),
            'x': tx[traj_valid].astype(float).tolist(),
            'y': ty[traj_valid].astype(float).tolist(),
            'z': tz[traj_valid].astype(float).tolist(),
        },
    }


def export_selected_frames(
    logfile,
    dataset,
    records,
    start_frame,
    end_frame,
    row_mode,
    use_manhattan,
    use_startup_filter,
    filter_aggressiveness,
    output_path=None,
):
    """Export the selected frame window to a JSON file."""
    start_frame, end_frame = clamp_frame_range(start_frame, end_frame, len(records))
    if output_path is None:
        stem = os.path.splitext(os.path.basename(logfile))[0]
        output_path = os.path.join(
            os.path.dirname(os.path.abspath(logfile)),
            f'{stem}_frames_{start_frame:05d}_{end_frame:05d}.json',
        )

    payload = build_export_payload(
        logfile,
        dataset,
        records,
        start_frame,
        end_frame,
        row_mode,
        use_manhattan,
        use_startup_filter,
        filter_aggressiveness,
    )
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)
    print(
        f"Exported {payload['meta']['point_count']:,} points from "
        f"frames {start_frame}–{end_frame} to {output_path}"
    )
    return output_path


def room_preset_path_for_log(logfile):
    """Return sidecar JSON path for saved room transforms of a log file."""
    abs_log = os.path.abspath(logfile)
    stem, _ = os.path.splitext(abs_log)
    return f'{stem}.room.json'


def load_room_preset(logfile):
    """Load a saved room preset for a log file if present."""
    preset_path = room_preset_path_for_log(logfile)
    if not os.path.exists(preset_path):
        return None, preset_path

    try:
        with open(preset_path, 'r', encoding='utf-8') as f:
            payload = json.load(f)
    except Exception as exc:
        print(f"Warning: could not read room preset {preset_path}: {exc}")
        return None, preset_path

    room = payload.get('room', payload)
    try:
        preset = {
            'width': float(room['width']),
            'length': float(room['length']),
            'height': float(room.get('height', 0.0)),
            'center_x': float(room.get('center_x', 0.0)),
            'center_y': float(room.get('center_y', 0.0)),
            'floor_z': float(room.get('floor_z', 0.0)),
            'yaw_deg': float(room.get('yaw_deg', 0.0)),
        }
    except (KeyError, TypeError, ValueError) as exc:
        print(f"Warning: invalid room preset {preset_path}: {exc}")
        return None, preset_path

    return preset, preset_path


def save_room_preset(logfile, room_state):
    """Persist the current room transform in a sidecar JSON next to the log."""
    preset_path = room_preset_path_for_log(logfile)
    payload = {
        'generated_by': 'plot_scan_3d.py',
        'source_log': os.path.abspath(logfile),
        'room': {
            'width': float(room_state['width']),
            'length': float(room_state['length']),
            'height': float(room_state['height']),
            'center_x': float(room_state['center_x']),
            'center_y': float(room_state['center_y']),
            'floor_z': float(room_state['floor_z']),
            'yaw_deg': float(room_state['yaw_deg']),
        },
    }
    with open(preset_path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)
        f.write('\n')
    return preset_path


# ─────────────────────────────────────────────────────────────────────────────
# 3‑D point computation  (mirrors compute_points but keeps Z + frame indices)
# ─────────────────────────────────────────────────────────────────────────────
def compute_points_3d(
    records,
    sensor_offsets_body_xy_m=None,
    sensor_yaw_trims_deg=None,
    column_az_offsets_deg=None,
    yaw_corrections_deg=None,
    filter_profile=None,
    row_filter_indices=None,
):
    """Return (px, py, pz, sensor_ids, frame_indices) – world‑frame 3‑D."""
    print("Computing 3D points …")
    all_x, all_y, all_z, all_sid = [], [], [], []
    frame_indices = [0]
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

    (elev_corrections, elev_angles_rad,
     azimuth_offsets, row_indices, col_indices) = build_projection_tables()

    total_pts = 0
    count = 0
    for rec_idx, rec in enumerate(records):
        if not frame_pose_is_usable(rec, filter_profile):
            frame_indices.append(total_pts)
            continue

        drone_x = rec['x']
        drone_y = rec['y']
        drone_z = rec['alt']
        corrected_yaw_deg = rec['yaw'] + yaw_corrections_deg[rec_idx]
        drone_yaw_rad = math.radians(corrected_yaw_deg)

        world_from_body = rotation_matrix_rz_ry_rx(
            drone_yaw_rad, rec['pitch'], rec['roll'],
        )

        measurements = extract_frame_measurements(
            rec, row_indices, col_indices,
            filter_profile['map_min_range_m'],
            filter_profile['map_max_range_m'],
            row_filter_indices=row_filter_indices,
        )
        if measurements is None:
            frame_indices.append(total_pts)
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
        body_elevation_rad = elev_angles_rad[zone_idx_in_sensor]

        # Full 3‑D ray in body frame
        ray_body = np.column_stack((
            np.cos(body_elevation_rad) * np.cos(body_azimuth_rad),
            np.cos(body_elevation_rad) * np.sin(body_azimuth_rad),
            np.sin(body_elevation_rad),
        ))
        sensor_fwd_comp = np.cos(body_elevation_rad) * np.cos(
            np.radians(corrected_azimuth_deg)
        )
        sensor_fwd_comp = np.clip(sensor_fwd_comp, 1e-4, None)
        scale = dists_m / sensor_fwd_comp

        sensor_offsets_body = np.column_stack((
            sensor_offsets_body_xy_m[sensor_idx, 0],
            sensor_offsets_body_xy_m[sensor_idx, 1],
            np.zeros(len(sensor_idx), dtype=np.float32),
        ))
        sensor_offsets_world = sensor_offsets_body @ world_from_body.T
        ray_world = ray_body @ world_from_body.T

        px = drone_x + sensor_offsets_world[:, 0] + scale * ray_world[:, 0]
        py = drone_y + sensor_offsets_world[:, 1] + scale * ray_world[:, 1]
        pz = drone_z + sensor_offsets_world[:, 2] + scale * ray_world[:, 2]

        # XY outlier rejection
        n = len(px)
        if n > OUTLIER_REJECT_MIN_NEIGHBORS:
            pts2d = np.column_stack((px, py))
            deltas = pts2d[:, None, :] - pts2d[None, :, :]
            dist2 = np.sum(deltas * deltas, axis=2)
            neighbours = np.count_nonzero(
                dist2 <= OUTLIER_REJECT_RADIUS_M ** 2, axis=1,
            ) - 1
            keep = neighbours >= OUTLIER_REJECT_MIN_NEIGHBORS
        else:
            keep = np.ones(n, dtype=bool)

        if not np.any(keep):
            frame_indices.append(total_pts)
            continue

        all_x.append(px[keep])
        all_y.append(py[keep])
        all_z.append(pz[keep])
        all_sid.append(sensor_idx[keep])
        total_pts += int(np.count_nonzero(keep))
        frame_indices.append(total_pts)
        count += 1

    if count == 0:
        empty = np.empty(0, np.float32)
        return empty, empty, empty, np.empty(0, np.uint8), frame_indices

    print(f"Generated {total_pts} 3D points from {count} frames.")
    return (
        np.concatenate(all_x).astype(np.float32),
        np.concatenate(all_y).astype(np.float32),
        np.concatenate(all_z).astype(np.float32),
        np.concatenate(all_sid).astype(np.uint8),
        frame_indices,
    )


def build_trajectory_3d(records, filter_profile):
    """Per-record trajectory arrays.  NaN for unusable frames."""
    n = len(records)
    xs = np.full(n, np.nan, dtype=np.float32)
    ys = np.full(n, np.nan, dtype=np.float32)
    zs = np.full(n, np.nan, dtype=np.float32)
    for i, r in enumerate(records):
        if frame_pose_is_usable(r, filter_profile):
            xs[i] = r['x']; ys[i] = r['y']; zs[i] = r['alt']
    return xs, ys, zs


# ─────────────────────────────────────────────────────────────────────────────
# Colour helpers
# ─────────────────────────────────────────────────────────────────────────────
def _turbo_ramp(t):
    """Attempt at a turbo-ish colourmap: dark-blue → cyan → green → yellow → red.

    *t* is a float array in [0, 1].  Returns (r, g, b) arrays in [0, 1].
    """
    # Piecewise-linear approximation (5 anchors)
    #   0.00 → (0.15, 0.05, 0.60)  dark blue
    #   0.25 → (0.05, 0.55, 0.85)  cyan
    #   0.50 → (0.10, 0.75, 0.20)  green
    #   0.75 → (0.90, 0.75, 0.05)  yellow
    #   1.00 → (0.80, 0.10, 0.10)  red
    anchors = np.array([
        [0.15, 0.05, 0.60],
        [0.05, 0.55, 0.85],
        [0.10, 0.75, 0.20],
        [0.90, 0.75, 0.05],
        [0.80, 0.10, 0.10],
    ], dtype=np.float32)
    idx_f = np.clip(t * 4.0, 0.0, 3.9999)
    lo = idx_f.astype(np.int32)
    frac = (idx_f - lo).reshape(-1, 1)
    rgb = anchors[lo] * (1.0 - frac) + anchors[lo + 1] * frac
    return rgb[:, 0], rgb[:, 1], rgb[:, 2]


def _build_vertex_colors(mode, n, vis_pz, vis_ux, vis_uz,
                          sf, ef, frame_indices, all_sids, all_pz):
    """Return list of (r,g,b,a) tuples for the current colour mode."""
    if n == 0:
        return []

    if mode == 'height':
        z_lo, z_hi = float(np.min(vis_pz)), float(np.max(vis_pz))
        span = z_hi - z_lo if (z_hi - z_lo) > 1e-6 else 1.0
        t = np.clip((vis_pz - z_lo) / span, 0.0, 1.0)
        r, g, b = _turbo_ramp(t)

    elif mode == 'depth':
        # XZ distance from centroid (how far from centre of the cloud)
        d = np.sqrt(vis_ux ** 2 + vis_uz ** 2)
        d_max = float(np.max(d)) if float(np.max(d)) > 1e-6 else 1.0
        t = np.clip(d / d_max, 0.0, 1.0)
        r, g, b = _turbo_ramp(t)

    elif mode == 'frame':
        # Colour by capture order (frame index → rainbow)
        sp = frame_indices[sf]
        ep = frame_indices[min(ef + 1, len(frame_indices) - 1)]
        total = max(ep - sp, 1)
        t = np.clip(np.arange(n, dtype=np.float32) / total, 0.0, 1.0)
        r, g, b = _turbo_ramp(t)

    else:  # 'black'
        r = np.full(n, 0.08, dtype=np.float32)
        g = np.full(n, 0.08, dtype=np.float32)
        b = np.full(n, 0.08, dtype=np.float32)

    a = [1.0] * n
    return list(zip(r.tolist(), g.tolist(), b.tolist(), a))


# ─────────────────────────────────────────────────────────────────────────────
# Ursina viewer
# ─────────────────────────────────────────────────────────────────────────────
def launch_viewer(records, max_points, logfile, initial_start_frame=0,
                  initial_end_frame=None, room_size=None, room_offset=(0.0, 0.0)):
    try:
        from ursina import (
            Ursina, Entity, Mesh, EditorCamera, Text, Button,
            destroy, color, time, mouse, held_keys,
        )
        from ursina.prefabs.slider import ThinSlider
    except ImportError:
        print("Error: ursina is required for the viewer. Install with: pip install ursina")
        sys.exit(1)

    num_records = len(records)
    initial_end_frame = num_records - 1 if initial_end_frame is None else initial_end_frame
    initial_start_frame, initial_end_frame = clamp_frame_range(
        initial_start_frame, initial_end_frame, num_records,
    )
    saved_room_preset, room_preset_path = load_room_preset(logfile)

    app = Ursina(
        borderless=False,
        title='3D Scan Viewer',
        vsync=True,
        editor_ui_enabled=False,
        development_mode=False,
    )

    # ── State ─────────────────────────────────────────────────────────────
    state = {
        'point_size': 4,
        'color_mode': 'sensor',
        'start_frame': initial_start_frame,
        'end_frame': initial_end_frame,
        # Filter / projection settings (these trigger full recompute)
        'use_manhattan': False,
        'use_startup_filter': True,
        'row_mode': 'all',
        'filter_aggressiveness': DEFAULT_FILTER_AGGRESSIVENESS,
        'live_traj': True,
        'screenshot_mode': False,
        'show_room': (room_size is not None or saved_room_preset is not None),
    }

    gui_entities = []

    def track_gui(entity):
        gui_entities.append(entity)
        return entity

    def set_screenshot_mode(enabled):
        state['screenshot_mode'] = bool(enabled)
        gui_visible = not state['screenshot_mode']
        for ent in gui_entities:
            ent.enabled = gui_visible
            ent.visible = gui_visible
        mouse.visible = gui_visible
        set_button_text(
            btn_screenshot,
            f'Screenshot: {"On" if state["screenshot_mode"] else "Off"}',
        )

    # Mutable point data – replaced by recompute_all()
    pdata = {
        'px': np.empty(0, np.float32), 'py': np.empty(0, np.float32),
        'pz': np.empty(0, np.float32), 'sids': np.empty(0, np.uint8),
        'frame_indices': [0] * (num_records + 1),
        'ux': np.empty(0, np.float32), 'uy': np.empty(0, np.float32),
        'uz': np.empty(0, np.float32),
        'cx': 0.0, 'cy': 0.0, 'cz': 0.0,
        'traj_x': np.empty(0, np.float32),
        'traj_y': np.empty(0, np.float32),
        'traj_z': np.empty(0, np.float32),
    }

    # ── Entities ──────────────────────────────────────────────────────────
    sensor_entities = []
    for sid in range(NUM_SENSORS):
        if not ENABLED_SENSOR_MASK[sid]:
            sensor_entities.append(None)
            continue
        e = Entity()
        e.setLightOff()
        sensor_entities.append(e)

    alt_entity = Entity(visible=False)
    alt_entity.setLightOff()

    traj_entity = Entity()
    traj_entity.setLightOff()

    floor_entity = Entity()           # parent for floor plane + grid lines
    floor_plane = Entity(
        parent=floor_entity,
        model='plane', scale=10,
        color=color.rgb(200, 200, 205),
    )
    floor_plane.setLightOff()
    floor_grid_entity = Entity(parent=floor_entity)
    floor_grid_entity.setLightOff()

    def rebuild_floor_grid(extent, y_pos):
        """Create a grid of lines on the floor plane."""
        floor_entity.y = y_pos
        floor_plane.scale = extent * 1.5

        # Remove old grid lines
        for c in list(floor_grid_entity.children):
            destroy(c)

        half = extent * 0.6
        spacing = 1.0        # 1‑metre grid
        grid_col = (0.25, 0.25, 0.28, 1.0)
        lines = []
        v = -half
        while v <= half + 1e-6:
            # line along X
            lines.append((v, 0, -half))
            lines.append((v, 0, half))
            # line along Z
            lines.append((-half, 0, v))
            lines.append((half, 0, v))
            v += spacing
        if lines:
            Entity(
                parent=floor_grid_entity,
                model=Mesh(vertices=lines, mode='line', thickness=1),
                color=color.rgba(120, 120, 130, 220),
            ).setLightOff()

    # ── Room boundary box ──────────────────────────────────────────────────
    room_entity = Entity(visible=state['show_room'])
    room_entity.setLightOff()
    room_wire_entity = {'entity': None}
    room_defaults = None
    room_state = None

    if room_size is not None:
        room_defaults = {
            'width': float(room_size[0]),
            'length': float(room_size[1]),
            'height': float(room_size[2]) if len(room_size) >= 3 else 0.0,
            'center_x': float(room_offset[0]),
            'center_y': float(room_offset[1]),
            'floor_z': 0.0,
            'yaw_deg': 0.0,
        }
        room_state = dict(room_defaults)
    elif saved_room_preset is not None:
        room_defaults = dict(saved_room_preset)
        room_state = dict(saved_room_preset)

    if room_state is not None and saved_room_preset is not None:
        room_state.update(saved_room_preset)
        room_defaults = dict(room_state)

    def build_room_wireframe_vertices():
        if room_state is None:
            return []

        width = max(0.05, float(room_state['width']))
        length = max(0.05, float(room_state['length']))
        height = max(0.0, float(room_state['height']))
        center_x = float(room_state['center_x'])
        center_y = float(room_state['center_y'])
        floor_z = float(room_state['floor_z'])
        yaw_rad = math.radians(float(room_state['yaw_deg']))
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        def world_to_viewer(wx, wy, wz):
            return (wx, wz, wy)

        corners_local = [
            (-width / 2.0, -length / 2.0),
            (width / 2.0, -length / 2.0),
            (width / 2.0, length / 2.0),
            (-width / 2.0, length / 2.0),
        ]

        corners_world = []
        for lx, ly in corners_local:
            wx = center_x + lx * cos_yaw - ly * sin_yaw
            wy = center_y + lx * sin_yaw + ly * cos_yaw
            corners_world.append((wx, wy))

        room_lines = []
        bottom = [world_to_viewer(wx, wy, floor_z) for wx, wy in corners_world]
        for idx in range(4):
            room_lines.extend((bottom[idx], bottom[(idx + 1) % 4]))

        if height > 0.0:
            top = [world_to_viewer(wx, wy, floor_z + height) for wx, wy in corners_world]
            for idx in range(4):
                room_lines.extend((top[idx], top[(idx + 1) % 4]))
                room_lines.extend((bottom[idx], top[idx]))

        return room_lines

    def sync_room_transform():
        room_entity.x = -pdata['cx']
        room_entity.y = -pdata['cz']
        room_entity.z = -pdata['cy']

    def sync_room_visibility():
        room_entity.visible = bool(room_state is not None and state['show_room'])

    def rebuild_room_mesh():
        if room_wire_entity['entity'] is not None:
            destroy(room_wire_entity['entity'])
            room_wire_entity['entity'] = None

        if room_state is None:
            sync_room_visibility()
            return

        room_lines = build_room_wireframe_vertices()
        if room_lines:
            room_wire_entity['entity'] = Entity(
                parent=room_entity,
                model=Mesh(vertices=room_lines, mode='line', thickness=2),
                color=color.rgba(0, 200, 210, 220),
            )
            room_wire_entity['entity'].setLightOff()

        sync_room_transform()
        sync_room_visibility()

    # ── Full recompute pipeline ───────────────────────────────────────────
    def recompute_all():
        hud_info.text = 'Recomputing …'
        dataset = prepare_scan_dataset(
            records,
            use_manhattan=state['use_manhattan'],
            use_startup_filter=state['use_startup_filter'],
            row_mode=state['row_mode'],
            filter_aggressiveness=state['filter_aggressiveness'],
        )

        pdata['px'] = dataset['px'];  pdata['py'] = dataset['py'];  pdata['pz'] = dataset['pz']
        pdata['sids'] = dataset['sids']
        pdata['frame_indices'] = dataset['frame_indices']
        pdata['cx'] = dataset['cx'];  pdata['cy'] = dataset['cy'];  pdata['cz'] = dataset['cz']
        pdata['ux'] = dataset['ux']
        pdata['uy'] = dataset['uy']
        pdata['uz'] = dataset['uz']
        pdata['traj_x'] = dataset['traj_x']
        pdata['traj_y'] = dataset['traj_y']
        pdata['traj_z'] = dataset['traj_z']

        # Build full trajectory line for non-playback display
        tx = pdata['traj_x']
        ty = pdata['traj_y']
        tz = pdata['traj_z']
        valid = np.isfinite(tx)
        if np.count_nonzero(valid) > 1:
            vx = tx[valid] - pdata['cx']
            vy = ty[valid] - pdata['cy']
            vz = tz[valid] - pdata['cz']
            step = max(1, len(vx) // 5000)
            tv = list(zip(
                vx[::step].tolist(), vz[::step].tolist(), vy[::step].tolist(),
            ))
            traj_entity.model = Mesh(vertices=tv, mode='line', thickness=2)
            traj_entity.color = color.red
            traj_entity.setLightOff()
            traj_entity.visible = True
        else:
            traj_entity.visible = False

        # Floor at lowest point altitude
        if len(pdata['pz']) > 0:
            ext = max(float(np.ptp(pdata['px'])), float(np.ptp(pdata['py'])))
            floor_y = float(np.min(pdata['pz']) - pdata['cz'])
            rebuild_floor_grid(ext, floor_y)

        sync_room_transform()

        # Clamp frame range to new data
        state['start_frame'], state['end_frame'] = clamp_frame_range(
            state['start_frame'], state['end_frame'], num_records,
        )
        start_slider.value = state['start_frame']
        end_slider.value = state['end_frame']
        start_label.text = f'Start: {state["start_frame"]}'
        end_label.text = f'End: {state["end_frame"]}'

        rebuild_cloud()

    # ── Rebuild visible point cloud from current frame range ──────────────
    def rebuild_cloud():
        sf = state['start_frame']
        # During playback, the playback head overrides the end frame
        if anim['playing'] and anim['playback_frame'] is not None:
            ef = anim['playback_frame']
        else:
            ef = state['end_frame']
        fi = pdata['frame_indices']
        sp = fi[sf]
        ep = fi[min(ef + 1, num_records)]

        vis_ux = pdata['ux'][sp:ep]
        vis_uy = pdata['uy'][sp:ep]
        vis_uz = pdata['uz'][sp:ep]
        vis_sid = pdata['sids'][sp:ep]
        vis_pz  = pdata['pz'][sp:ep]

        n = len(vis_ux)
        if n > max_points:
            step = math.ceil(n / max_points)
            vis_ux  = vis_ux[::step]
            vis_uy  = vis_uy[::step]
            vis_uz  = vis_uz[::step]
            vis_sid = vis_sid[::step]
            vis_pz  = vis_pz[::step]
            n = len(vis_ux)

        mode = state['color_mode']

        if mode == 'sensor':
            # Per-sensor entities with solid colour
            alt_entity.visible = False
            for sid in range(NUM_SENSORS):
                ent = sensor_entities[sid]
                if ent is None:
                    continue
                mask = vis_sid == sid
                cnt = int(np.count_nonzero(mask))
                if cnt == 0:
                    ent.visible = False
                    continue
                verts = list(zip(
                    vis_ux[mask].tolist(),
                    vis_uy[mask].tolist(),
                    vis_uz[mask].tolist(),
                ))
                r, g, b = SENSOR_RGB[sid]
                cols = [(r, g, b, 1.0)] * cnt
                ent.model = Mesh(vertices=verts, colors=cols, mode='point',
                                 thickness=state['point_size'],
                                 render_points_in_3d=False)
                ent.setLightOff()
                ent.setRenderModeThickness(state['point_size'])
                ent.visible = True
        else:
            # All other modes use the single alt_entity with vertex colours
            for ent in sensor_entities:
                if ent is not None:
                    ent.visible = False
            verts = list(zip(
                vis_ux.tolist(), vis_uy.tolist(), vis_uz.tolist(),
            ))
            cols = _build_vertex_colors(mode, n, vis_pz, vis_ux, vis_uz,
                                        sf, ef, pdata['frame_indices'],
                                        pdata['sids'], pdata['pz'])
            if n > 0:
                alt_entity.model = Mesh(
                    vertices=verts, colors=cols, mode='point',
                    thickness=state['point_size'],
                    render_points_in_3d=False,
                )
                alt_entity.setLightOff()
                alt_entity.setRenderModeThickness(state['point_size'])
            alt_entity.visible = n > 0

        # ── Live trajectory during playback ──────────────────────────────
        if anim['playing'] and anim['playback_frame'] is not None and state['live_traj']:
            tx = pdata['traj_x'][:ef + 1]
            ty = pdata['traj_y'][:ef + 1]
            tz = pdata['traj_z'][:ef + 1]
            cx, cy, cz = pdata['cx'], pdata['cy'], pdata['cz']
            valid = np.isfinite(tx)
            if np.count_nonzero(valid) > 1:
                vx = tx[valid] - cx;  vy = ty[valid] - cy;  vz = tz[valid] - cz
                tv = list(zip(vx.tolist(), vz.tolist(), vy.tolist()))
                traj_entity.model = Mesh(vertices=tv, mode='line', thickness=2)
                traj_entity.color = color.red
                traj_entity.setLightOff()
                traj_entity.visible = True
            else:
                traj_entity.visible = False

        hud_info.text = (
            f'Points: {n:,}   Colour: {mode}   Size: {state["point_size"]}   '
            f'Frames: {sf}–{ef}   {row_mode_detail(state["row_mode"])}'
        )

    # ── Camera ────────────────────────────────────────────────────────────
    ec = EditorCamera(rotation_smoothing=2, zoom_speed=2)
    ec.rotation_x = 35
    ec.rotation_y = -30

    # ── HUD ───────────────────────────────────────────────────────────────
    hud_color = color.rgb(20, 20, 20)
    track_gui(Text(
        text=(
            'RMB: orbit   MMB: pan   Scroll: zoom\n'
            '1/2: point size   C: colour   T: traj   G: ground   R: room\n'
            'J/L,I/K,Y/N move room   U/O rotate   Shift+same resize   Shift+R reset\n'
            'P or Save Room: save room   Space: play/pause   S: anim speed   E: export JSON   H: screenshot mode'
        ),
        position=(-0.86, 0.49), scale=0.7, color=hud_color,
    ))
    hud_info = track_gui(
        Text(text='', position=(-0.86, 0.39), scale=0.7, color=hud_color)
    )
    room_info = track_gui(
        Text(text='', position=(-0.86, 0.31), scale=0.7, color=hud_color)
    )

    def update_room_info():
        if room_state is None:
            room_info.text = 'Room: not loaded'
            return
        room_info.text = (
            f'Room: {"On" if state["show_room"] else "Hidden"}   '
            f'{room_state["width"]:.2f} x {room_state["length"]:.2f} x {room_state["height"]:.2f} m   '
            f'Center: ({room_state["center_x"]:.2f}, {room_state["center_y"]:.2f})   '
            f'Base Z: {room_state["floor_z"]:.2f}   '
            f'Yaw: {room_state["yaw_deg"]:.1f} deg   '
            f'Save: {os.path.basename(room_preset_path)}'
        )

    def save_current_room_preset():
        if room_state is None:
            hud_info.text = 'No room loaded to save.'
            return
        out_path = save_room_preset(logfile, room_state)
        hud_info.text = f'Saved room preset to {os.path.basename(out_path)}'

    for i, name in enumerate(SENSOR_NAMES):
        if not ENABLED_SENSOR_MASK[i]:
            continue
        r, g, b = SENSOR_RGB[i]
        track_gui(Text(
            text=f'# {name}',
            position=(0.72, 0.49 - i * 0.035), scale=0.7,
            color=color.rgb(int(r * 255), int(g * 255), int(b * 255)),
        ))

    # ── Toggle buttons (trigger full recompute) ──────────────────────────
    btn_w = 0.20
    btn_h = 0.035
    btn_y = -0.28
    btn_gap = 0.005
    btn_x0 = -0.86

    def _make_btn(txt, x, y):
        b = Button(
            text='', scale=(btn_w, btn_h), position=(x, y),
            color=color.rgb(30, 55, 90),
            highlight_color=color.rgb(50, 80, 120),
        )
        track_gui(b)
        b.label_entity = track_gui(Text(
            text=txt,
            position=(x, y + 0.002),
            origin=(0, 0),
            scale=0.6,
            color=color.white,
        ))
        return b

    def set_button_text(button, value):
        button.label_entity.text = value

    btn_manhattan = _make_btn(
        'Manhattan: Off', btn_x0 + btn_w / 2, btn_y,
    )
    btn_startup = _make_btn(
        'Startup: On', btn_x0 + btn_w + btn_gap + btn_w / 2, btn_y,
    )
    btn_rows = _make_btn(
        'Rows: All', btn_x0 + 2 * (btn_w + btn_gap) + btn_w / 2, btn_y,
    )
    btn_live_traj = _make_btn(
        'Live Traj: On', btn_x0 + 3 * (btn_w + btn_gap) + btn_w / 2, btn_y,
    )
    btn_export = _make_btn(
        'Export JSON', btn_x0 + 4 * (btn_w + btn_gap) + btn_w / 2, btn_y,
    )
    btn_screenshot = _make_btn(
        'Screenshot: Off', btn_x0 + 5 * (btn_w + btn_gap) + btn_w / 2, btn_y,
    )
    btn_save_room = _make_btn(
        'Save Room', 0.72, -0.38,
    )

    def toggle_manhattan():
        state['use_manhattan'] = not state['use_manhattan']
        set_button_text(
            btn_manhattan,
            f'Manhattan: {"On" if state["use_manhattan"] else "Off"}',
        )
        recompute_all()

    def toggle_startup():
        state['use_startup_filter'] = not state['use_startup_filter']
        set_button_text(
            btn_startup,
            f'Startup: {"On" if state["use_startup_filter"] else "Off"}',
        )
        recompute_all()

    def cycle_rows():
        state['row_mode'] = next_row_mode_3d(state['row_mode'])
        set_button_text(btn_rows, f'Rows: {row_mode_display(state["row_mode"])}')
        recompute_all()

    def toggle_live_traj():
        state['live_traj'] = not state['live_traj']
        set_button_text(
            btn_live_traj,
            f'Live Traj: {"On" if state["live_traj"] else "Off"}',
        )

    def export_current_selection():
        out_path = export_selected_frames(
            logfile,
            pdata,
            records,
            state['start_frame'],
            state['end_frame'],
            row_mode=state['row_mode'],
            use_manhattan=state['use_manhattan'],
            use_startup_filter=state['use_startup_filter'],
            filter_aggressiveness=state['filter_aggressiveness'],
            output_path=None,
        )
        hud_info.text = (
            f'Exported frames {state["start_frame"]}–{state["end_frame"]} '
            f'to {os.path.basename(out_path)}'
        )

    def toggle_screenshot_mode():
        set_screenshot_mode(not state['screenshot_mode'])

    btn_manhattan.on_click = toggle_manhattan
    btn_startup.on_click = toggle_startup
    btn_rows.on_click = cycle_rows
    btn_live_traj.on_click = toggle_live_traj
    btn_export.on_click = export_current_selection
    btn_screenshot.on_click = toggle_screenshot_mode
    btn_save_room.on_click = save_current_room_preset

    # ── Filter aggressiveness slider ──────────────────────────────────────
    filter_label = track_gui(Text(
        text=f'Filter: {DEFAULT_FILTER_AGGRESSIVENESS:.2f}',
        position=(-0.86, -0.325), scale=0.7, color=hud_color,
    ))
    filter_slider = track_gui(ThinSlider(
        min=0, max=2.0, default=DEFAULT_FILTER_AGGRESSIVENESS,
        step=0.05, dynamic=False,
        position=(-0.2, -0.335), scale=(0.7, 1),
    ))

    def on_filter_changed():
        val = round(filter_slider.value, 2)
        state['filter_aggressiveness'] = val
        filter_label.text = f'Filter: {val:.2f}'
        recompute_all()

    filter_slider.on_value_changed = on_filter_changed

    # ── Frame-range sliders ───────────────────────────────────────────────
    slider_max = max(num_records - 1, 1)

    start_label = track_gui(Text(
        text=f'Start: {state["start_frame"]}',
        position=(-0.86, -0.38), scale=0.7, color=hud_color,
    ))
    start_slider = track_gui(ThinSlider(
        min=0, max=slider_max, default=state['start_frame'], step=1, dynamic=False,
        position=(-0.2, -0.39), scale=(0.7, 1),
    ))

    end_label = track_gui(Text(
        text=f'End: {state["end_frame"]}',
        position=(-0.86, -0.43), scale=0.7, color=hud_color,
    ))
    end_slider = track_gui(ThinSlider(
        min=0, max=slider_max, default=state['end_frame'], step=1, dynamic=False,
        position=(-0.2, -0.44), scale=(0.7, 1),
    ))

    def on_start_changed():
        val = int(round(start_slider.value))
        if val > state['end_frame']:
            state['end_frame'] = val
            end_slider.value = val
            end_label.text = f'End: {val}'
        state['start_frame'] = val
        start_label.text = f'Start: {val}'
        rebuild_cloud()

    def on_end_changed():
        val = int(round(end_slider.value))
        if val < state['start_frame']:
            state['start_frame'] = val
            start_slider.value = val
            start_label.text = f'Start: {val}'
        state['end_frame'] = val
        end_label.text = f'End: {val}'
        rebuild_cloud()

    start_slider.on_value_changed = on_start_changed
    end_slider.on_value_changed = on_end_changed

    # ── Frame animation ──────────────────────────────────────────────────
    # Space plays from Start slider to End slider.  A "playback head"
    # sweeps from start → end; rebuild_cloud uses it as the effective end.
    ANIM_SPEEDS = [1, 2, 5, 10, 25, 50]
    anim = {
        'playing': False,
        'speed_idx': 2,             # index into ANIM_SPEEDS (default 5 fps)
        'accum': 0.0,
        'playback_frame': None,     # current head position (None = not animating)
    }

    anim_label = track_gui(Text(
        text='',
        position=(0.40, -0.43), scale=0.7, color=hud_color,
    ))

    def anim_update_label():
        fps = ANIM_SPEEDS[anim['speed_idx']]
        if anim['playing']:
            pf = anim['playback_frame'] or state['start_frame']
            anim_label.text = (
                f'Playing {pf}/{state["end_frame"]}  {fps} fr/s   '
                f'[Space]=pause  [S]=speed'
            )
        else:
            anim_label.text = f'[Space]=play  [S]=speed ({fps} fr/s)'

    def restore_full_trajectory():
        """Rebuild full trajectory line after playback ends."""
        tx, ty, tz = pdata['traj_x'], pdata['traj_y'], pdata['traj_z']
        cx, cy, cz = pdata['cx'], pdata['cy'], pdata['cz']
        valid = np.isfinite(tx)
        if np.count_nonzero(valid) > 1:
            vx = tx[valid] - cx;  vy = ty[valid] - cy;  vz = tz[valid] - cz
            step = max(1, len(vx) // 5000)
            tv = list(zip(
                vx[::step].tolist(), vz[::step].tolist(), vy[::step].tolist(),
            ))
            traj_entity.model = Mesh(vertices=tv, mode='line', thickness=2)
            traj_entity.color = color.red
            traj_entity.setLightOff()
            traj_entity.visible = True

    def anim_toggle():
        if not anim['playing']:
            # Start: playback head begins at start slider
            anim['playing'] = True
            anim['playback_frame'] = state['start_frame']
            anim['accum'] = 0.0
            rebuild_cloud()
        else:
            # Pause: stop advancing, snap view back to full slider range
            anim['playing'] = False
            anim['playback_frame'] = None
            restore_full_trajectory()
            rebuild_cloud()
        anim_update_label()

    def anim_cycle_speed():
        anim['speed_idx'] = (anim['speed_idx'] + 1) % len(ANIM_SPEEDS)
        anim_update_label()

    class AnimUpdater(Entity):
        """Advance playback head each tick while playing."""
        def update(self):
            if not anim['playing']:
                return
            fps = ANIM_SPEEDS[anim['speed_idx']]
            anim['accum'] += time.dt
            step = 1.0 / fps
            if anim['accum'] < step:
                return
            frames_to_advance = int(anim['accum'] / step)
            anim['accum'] -= frames_to_advance * step

            new_head = anim['playback_frame'] + frames_to_advance
            target_end = state['end_frame']
            if new_head >= target_end:
                new_head = target_end
                anim['playing'] = False
                anim['playback_frame'] = None
                restore_full_trajectory()

            if anim['playing']:
                anim['playback_frame'] = new_head
            rebuild_cloud()
            anim_update_label()

    AnimUpdater()
    anim_update_label()

    class RoomTransformController(Entity):
        """Interactive move/resize/rotate controls for the room wireframe."""
        def update(self):
            if room_state is None:
                return

            changed = False
            move_step = 1.0 * time.dt
            vertical_step = 0.8 * time.dt
            rotate_step = 60.0 * time.dt
            resize_step = 1.0 * time.dt
            shift_held = held_keys['shift']

            if shift_held:
                if held_keys['j']:
                    room_state['width'] = max(0.05, room_state['width'] - resize_step)
                    changed = True
                if held_keys['l']:
                    room_state['width'] += resize_step
                    changed = True
                if held_keys['i']:
                    room_state['length'] += resize_step
                    changed = True
                if held_keys['k']:
                    room_state['length'] = max(0.05, room_state['length'] - resize_step)
                    changed = True
                if held_keys['u']:
                    room_state['height'] = max(0.0, room_state['height'] - resize_step)
                    changed = True
                if held_keys['o']:
                    room_state['height'] += resize_step
                    changed = True
            else:
                if held_keys['j']:
                    room_state['center_x'] -= move_step
                    changed = True
                if held_keys['l']:
                    room_state['center_x'] += move_step
                    changed = True
                if held_keys['i']:
                    room_state['center_y'] += move_step
                    changed = True
                if held_keys['k']:
                    room_state['center_y'] -= move_step
                    changed = True
                if held_keys['y']:
                    room_state['floor_z'] += vertical_step
                    changed = True
                if held_keys['n']:
                    room_state['floor_z'] -= vertical_step
                    changed = True
                if held_keys['u']:
                    room_state['yaw_deg'] -= rotate_step
                    changed = True
                if held_keys['o']:
                    room_state['yaw_deg'] += rotate_step
                    changed = True

            if changed:
                rebuild_room_mesh()
                update_room_info()

    RoomTransformController()

    # ── Keyboard input ────────────────────────────────────────────────────
    class KeyHandler(Entity):
        def input(self, key):
            if key == '1':
                state['point_size'] = max(1, state['point_size'] - 1)
                for ent in sensor_entities:
                    if ent is not None and ent.visible:
                        ent.setRenderModeThickness(state['point_size'])
                if alt_entity.visible:
                    alt_entity.setRenderModeThickness(state['point_size'])
                hud_info.text = (
                    f'Size: {state["point_size"]}   '
                    f'Mode: {state["color_mode"]}   '
                    f'Frames: {state["start_frame"]}–{state["end_frame"]}'
                )
            elif key == '2':
                state['point_size'] = min(12, state['point_size'] + 1)
                for ent in sensor_entities:
                    if ent is not None and ent.visible:
                        ent.setRenderModeThickness(state['point_size'])
                if alt_entity.visible:
                    alt_entity.setRenderModeThickness(state['point_size'])
                hud_info.text = (
                    f'Size: {state["point_size"]}   '
                    f'Mode: {state["color_mode"]}   '
                    f'Frames: {state["start_frame"]}–{state["end_frame"]}'
                )
            elif key == 'c':
                idx = COLOR_MODES.index(state['color_mode'])
                state['color_mode'] = COLOR_MODES[(idx + 1) % len(COLOR_MODES)]
                rebuild_cloud()
            elif key == 'h':
                toggle_screenshot_mode()
            elif key == 't':
                traj_entity.visible = not traj_entity.visible
            elif key == 'g':
                floor_entity.visible = not floor_entity.visible
            elif key == 'space':
                anim_toggle()
            elif key == 's':
                anim_cycle_speed()
            elif key == 'r':
                if room_state is not None and held_keys['shift'] and room_defaults is not None:
                    room_state.update(room_defaults)
                    rebuild_room_mesh()
                else:
                    state['show_room'] = not state['show_room']
                    sync_room_visibility()
                update_room_info()
            elif key == 'p':
                save_current_room_preset()
            elif key == 'e':
                export_current_selection()

    KeyHandler()

    # ── Initial compute + render ──────────────────────────────────────────
    rebuild_room_mesh()
    recompute_all()
    update_room_info()
    set_screenshot_mode(False)
    app.run()


# ─────────────────────────────────────────────────────────────────────────────
# CLI entry
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="3D ToF Scan Log Viewer (Ursina)")
    parser.add_argument("logfile", nargs='?', help="Path to .bin log file")
    parser.add_argument("--max-points", type=int, default=MAX_DISPLAY_POINTS,
                        help=f"Max points to render (default {MAX_DISPLAY_POINTS})")
    parser.add_argument("--start-frame", type=int, default=0,
                        help="First frame to display/export (default 0)")
    parser.add_argument("--end-frame", type=int, default=None,
                        help="Last frame to display/export (default last frame)")
    parser.add_argument(
        "--export-json", nargs='?', const='',
        help="Export selected frames to JSON. Optionally provide an output path.",
    )
    parser.add_argument(
        "--no-viewer", action='store_true',
        help="Skip the Ursina viewer. Useful with --export-json on headless systems.",
    )
    parser.add_argument(
        "--room",
        default=None,
        help="Room boundary as WxL or WxLxH in metres (e.g. '1.8x2.7x2.4'). Draws a wireframe box.",
    )
    parser.add_argument(
        "--room-offset",
        default=None,
        help="Room centre offset as X,Y in metres (e.g. '0.5,0.3'). Default: 0,0.",
    )
    args = parser.parse_args()

    room_size = None
    room_offset = (0.0, 0.0)
    if args.room:
        parts = args.room.split('x')
        room_size = tuple(float(p) for p in parts)
    if args.room_offset:
        parts = args.room_offset.split(',')
        if len(parts) >= 2:
            room_offset = (float(parts[0]), float(parts[1]))

    filename = args.logfile
    if not filename:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk(); root.withdraw()
            filename = filedialog.askopenfilename(
                title="Select Scan Log",
                filetypes=(("Binary Log", "*.bin"), ("All files", "*.*")),
            )
            root.destroy()
        except Exception:
            pass
    if not filename:
        print("No file selected."); sys.exit(0)

    records = parse_log(filename)
    if not records:
        print("No data found."); sys.exit(1)

    start_frame, end_frame = clamp_frame_range(
        args.start_frame,
        len(records) - 1 if args.end_frame is None else args.end_frame,
        len(records),
    )

    if args.export_json is not None:
        dataset = prepare_scan_dataset(records)
        export_selected_frames(
            filename,
            dataset,
            records,
            start_frame,
            end_frame,
            row_mode='all',
            use_manhattan=False,
            use_startup_filter=True,
            filter_aggressiveness=DEFAULT_FILTER_AGGRESSIVENESS,
            output_path=args.export_json or None,
        )

    if args.no_viewer:
        if args.export_json is None:
            print("--no-viewer was set, but nothing was exported.")
        return

    launch_viewer(
        records,
        args.max_points,
        filename,
        initial_start_frame=start_frame,
        initial_end_frame=end_frame,
        room_size=room_size,
        room_offset=room_offset,
    )


if __name__ == '__main__':
    main()
