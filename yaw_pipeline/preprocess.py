"""Shared image preprocessing for training, conversion, and deployment.

Keeping this in one module prevents train/deploy skew.  The default ``stretch``
mode reproduces the original pipeline; ``center_crop`` preserves geometry when
converting a 4:3 camera frame to a square model input.
"""
from __future__ import annotations

from typing import Union

import cv2
import numpy as np


ImageSource = Union[str, np.ndarray]


def read_gray(src: ImageSource) -> np.ndarray:
    if isinstance(src, np.ndarray):
        if src.ndim == 2:
            return src
        return cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    image = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(src)
    return image


def resize_gray(image: np.ndarray, size: int, mode: str = "stretch") -> np.ndarray:
    """Return a square uint8 image using a documented geometric policy."""
    if image.ndim != 2:
        raise ValueError(f"expected grayscale HxW image, got shape {image.shape}")
    h, w = image.shape
    if mode == "stretch":
        out = image
    elif mode == "center_crop":
        side = min(h, w)
        y0 = (h - side) // 2
        x0 = (w - side) // 2
        out = image[y0:y0 + side, x0:x0 + side]
    elif mode == "letterbox":
        scale = min(size / w, size / h)
        nw, nh = max(1, round(w * scale)), max(1, round(h * scale))
        resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
        out = np.zeros((size, size), dtype=np.uint8)
        y0, x0 = (size - nh) // 2, (size - nw) // 2
        out[y0:y0 + nh, x0:x0 + nw] = resized
        return out
    else:
        raise ValueError(f"unknown resize mode: {mode}")

    if out.shape != (size, size):
        interpolation = cv2.INTER_AREA if max(out.shape) > size else cv2.INTER_LINEAR
        out = cv2.resize(out, (size, size), interpolation=interpolation)
    return out.astype(np.uint8, copy=False)


def load_gray(src: ImageSource, size: int, mode: str = "stretch") -> np.ndarray:
    return resize_gray(read_gray(src), size, mode)


def load_pair_float(pair: dict, size: int, mode: str = "stretch") -> np.ndarray:
    """Return an NCHW float32 pair in [0,1], shape [1,2,size,size]."""
    a = load_gray(pair["img_a"], size, mode).astype(np.float32) / 255.0
    b = load_gray(pair["img_b"], size, mode).astype(np.float32) / 255.0
    return np.stack([a, b], axis=0)[None]
