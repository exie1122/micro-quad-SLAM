"""
Data-source abstraction for the yaw-correction pipeline.

This is the whole point of the dry run: every downstream stage (train, convert,
eval, compare) consumes ONLY the `YawDataset` interface below. Swapping the
public-dataset adapter for your own `RealRigDataset` is the only change needed to
run on real hardware data.

Interface
---------
    YawDataset.sequences() -> list[YawSequence]   # whole recordings
    YawSequence            -> iterable of FramePair
    FramePair              -> two frames + GT relative yaw + optional gyro rate

A FramePair carries ground-truth relative yaw (radians, camera vertical axis; see
geometry.py) and a per-pair gyro yaw-rate (rad/s) which may be REAL or SIMULATED.
`YawDataset.gyro_is_simulated` says which.
"""
from __future__ import annotations
import os
import zlib
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Union, List

import numpy as np
from geometry import quat_to_matrix, relative_yaw
from preprocess import load_gray


def _load_gray(src: Union[str, np.ndarray], size: int) -> np.ndarray:
    """Load `src` (path or HxW array) as a size×size uint8 grayscale image."""
    return load_gray(src, size, "stretch")


@dataclass
class FramePair:
    seq: str
    idx: int
    t_a: float
    t_b: float
    rel_yaw: float                              # GROUND-TRUTH relative yaw (rad)
    gyro_yaw_rate: Optional[float] = None       # rad/s about vertical
    img_a: Union[str, np.ndarray, None] = None  # path or array
    img_b: Union[str, np.ndarray, None] = None

    @property
    def dt(self) -> float:
        return self.t_b - self.t_a

    def load_pair(self, size: int = 128) -> np.ndarray:
        """Return stacked frame pair as [2, size, size] uint8 (channel 0 = a, 1 = b)."""
        a = _load_gray(self.img_a, size)
        b = _load_gray(self.img_b, size)
        return np.stack([a, b], axis=0)


@dataclass
class YawSequence:
    name: str
    pairs: List[FramePair] = field(default_factory=list)
    def __iter__(self):
        return iter(self.pairs)
    def __len__(self):
        return len(self.pairs)


class YawDataset(ABC):
    """Adapter base class. Implement `sequences()` and set `gyro_is_simulated`."""
    name: str = "abstract"
    gyro_is_simulated: bool = False

    @abstractmethod
    def sequences(self) -> List[YawSequence]:
        ...


# ---------------------------------------------------------------------------
# PUBLIC-DATASET ADAPTER:  TUM RGB-D  (camera + GT pose; gyro is SIMULATED)
# ---------------------------------------------------------------------------
def _read_tum_list(path):
    """Read a TUM 'timestamp value...' file, skipping comments. Returns (ts, rows)."""
    ts, rows = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            ts.append(float(parts[0]))
            rows.append(parts[1:])
    return np.array(ts), rows


def _associate(src_ts, dst_ts, max_dt=0.02):
    """For each src timestamp, index of nearest dst within max_dt, else -1."""
    idx = np.searchsorted(dst_ts, src_ts)
    out = np.full(len(src_ts), -1, dtype=int)
    for i, t in enumerate(src_ts):
        j = idx[i]
        cands = [k for k in (j - 1, j) if 0 <= k < len(dst_ts)]
        if not cands:
            continue
        best = min(cands, key=lambda k: abs(dst_ts[k] - t))
        if abs(dst_ts[best] - t) <= max_dt:
            out[i] = best
    return out


class TumYawDataset(YawDataset):
    """
    TUM RGB-D adapter. Each sequence dir must contain rgb.txt, groundtruth.txt and
    the rgb/*.png images (standard TUM layout). Gyro is SIMULATED from GT angular
    rate plus a constant bias and Gaussian noise (clearly labeled downstream).
    """
    name = "tum_rgbd"
    gyro_is_simulated = True

    def __init__(self, seq_dirs: dict, frame_stride: int = 1, img_size: int = 128,
                 gyro_bias_deg_s: float = 0.8, gyro_noise_deg_s: float = 0.15,
                 assoc_max_dt: float = 0.02, seed: int = 0):
        self.seq_dirs = seq_dirs            # {name: path}
        self.frame_stride = frame_stride
        self.img_size = img_size
        self.gyro_bias = np.deg2rad(gyro_bias_deg_s)
        self.gyro_noise = np.deg2rad(gyro_noise_deg_s)
        self.assoc_max_dt = assoc_max_dt
        self.seed = seed

    def _build_sequence(self, name, path) -> YawSequence:
        rgb_ts, rgb_rows = _read_tum_list(os.path.join(path, "rgb.txt"))
        gt_ts, gt_rows = _read_tum_list(os.path.join(path, "groundtruth.txt"))
        assoc = _associate(rgb_ts, gt_ts, self.assoc_max_dt)

        # rotation matrix per rgb frame (None where no GT)
        R = [None] * len(rgb_ts)
        for i, j in enumerate(assoc):
            if j >= 0:
                tx, ty, tz, qx, qy, qz, qw = map(float, gt_rows[j])
                R[i] = quat_to_matrix(qx, qy, qz, qw)

        # Python's built-in hash is randomized between processes; CRC32 keeps
        # simulated-noise runs reproducible across machines and invocations.
        rng = np.random.default_rng(self.seed + zlib.crc32(name.encode("utf-8")) % 10000)
        s = self.frame_stride
        pairs = []
        for i in range(0, len(rgb_ts) - s):
            ja, jb = i, i + s
            if R[ja] is None or R[jb] is None:
                continue
            dt = rgb_ts[jb] - rgb_ts[ja]
            if dt <= 0:
                continue
            ryaw = relative_yaw(R[ja], R[jb])
            gyro_rate = ryaw / dt + self.gyro_bias + rng.normal(0, self.gyro_noise)
            pairs.append(FramePair(
                seq=name, idx=i, t_a=float(rgb_ts[ja]), t_b=float(rgb_ts[jb]),
                rel_yaw=ryaw, gyro_yaw_rate=float(gyro_rate),
                img_a=os.path.join(path, rgb_rows[ja][0]),
                img_b=os.path.join(path, rgb_rows[jb][0]),
            ))
        return YawSequence(name=name, pairs=pairs)

    def sequences(self) -> List[YawSequence]:
        return [self._build_sequence(n, p) for n, p in self.seq_dirs.items()]


# ---------------------------------------------------------------------------
# STUB ADAPTER for your real captured data -- FILL THIS IN LATER.
# ---------------------------------------------------------------------------
class RealRigDataset(YawDataset):
    """
    >>> STUB: implement this with your own rig's data. <<<

    Once implemented, the ENTIRE pipeline (train/convert/eval/compare) runs
    unchanged -- that is the purpose of the dry run.

    Expected per sequence, you must provide for each consecutive frame pair:
      - img_a, img_b : grayscale frames (paths or HxW uint8 arrays)
      - rel_yaw      : GROUND-TRUTH relative yaw in radians, using the same
                       convention as geometry.relative_yaw (camera vertical axis).
                       Derive this from your motion-capture / RTK / fused pose.
      - gyro_yaw_rate: REAL gyro angular rate about the vertical axis (rad/s),
                       time-synced to frame b. Set `gyro_is_simulated = False`.
      - t_a, t_b     : frame timestamps (s)

    Suggested layout (adapt freely):
        <root>/<sequence>/frames/*.png
        <root>/<sequence>/imu.csv         # t, gx, gy, gz, ...
        <root>/<sequence>/pose_gt.csv     # t, tx,ty,tz, qx,qy,qz,qw

    Implementation sketch:
        1. parse pose_gt.csv -> rotation matrices; associate to frames by timestamp
        2. rel_yaw = geometry.relative_yaw(R_a, R_b)
        3. associate imu.csv to frame b; gyro_yaw_rate = component about vertical
        4. emit FramePair(...) per pair, group into YawSequence per recording
    """
    name = "real_rig"
    gyro_is_simulated = False  # you will provide REAL gyro

    def __init__(self, root: str, **kwargs):
        self.root = root
        self.kwargs = kwargs

    def sequences(self) -> List[YawSequence]:
        raise NotImplementedError(
            "RealRigDataset is a stub. Implement sequences() with your captured "
            "frames + ground-truth pose + real gyro. See the class docstring. "
            "No downstream code needs to change."
        )
