#!/usr/bin/env python3
"""
Build a tiny conv net (stand-in for a yaw/rotation regressor) and export to ONNX.
Also generates a small calibration image set for TPU-MLIR INT8 quantization.

Input : 1x1x128x128 grayscale
Output: 3 floats (e.g. sin(yaw), cos(yaw), confidence) -- a tiny regression head.

Kept deliberately small so it fits the SG2002 NPU and the board's ~87 MB free RAM.
Runs on an x86 host (where torch is available); only the resulting .cvimodel
goes to the board.
"""
import os
import numpy as np
import torch
import torch.nn as nn

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
ONNX_PATH = os.path.join(OUT_DIR, "tiny_yaw_cnn.onnx")
CALIB_DIR = os.path.join(OUT_DIR, "calib_images")


class TinyYawCNN(nn.Module):
    """Small conv stack + global pool + tiny FC head."""
    def __init__(self, n_out=3):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 8, 3, stride=2, padding=1),   # 128 -> 64
            nn.ReLU(inplace=True),
            nn.Conv2d(8, 16, 3, stride=2, padding=1),  # 64 -> 32
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), # 32 -> 16
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 32, 3, stride=2, padding=1), # 16 -> 8
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1),                   # 32 x 1 x 1
        )
        self.head = nn.Linear(32, n_out)

    def forward(self, x):
        x = self.features(x)
        x = torch.flatten(x, 1)
        return self.head(x)


def make_calib_images(n=60, size=128):
    """Synthetic grayscale frames resembling the ORB test frames (PNG, 0-255)."""
    import cv2
    os.makedirs(CALIB_DIR, exist_ok=True)
    rng = np.random.default_rng(123)
    paths = []
    for k in range(n):
        img = rng.integers(0, 60, size=(size, size), dtype=np.uint8)
        for _ in range(40):
            x1, y1 = rng.integers(0, size), rng.integers(0, size)
            x2, y2 = rng.integers(0, size), rng.integers(0, size)
            val = int(rng.integers(80, 255))
            if k % 3 == 0:
                cv2.rectangle(img, (x1, y1), (x2, y2), val, 1)
            elif k % 3 == 1:
                cv2.circle(img, (x1, y1), int(rng.integers(3, 20)), val, 1)
            else:
                cv2.line(img, (x1, y1), (x2, y2), val, 1)
        p = os.path.join(CALIB_DIR, f"calib_{k:03d}.png")
        cv2.imwrite(p, img)
        paths.append(p)
    with open(os.path.join(OUT_DIR, "calib_list.txt"), "w") as f:
        f.write("\n".join(paths) + "\n")
    return len(paths)


def main():
    torch.manual_seed(0)
    model = TinyYawCNN()
    model.train(False)  # inference mode (equivalent to .eval())
    n_params = sum(p.numel() for p in model.parameters())
    dummy = torch.randn(1, 1, 128, 128)
    with torch.no_grad():
        out = model(dummy)
    torch.onnx.export(
        model, dummy, ONNX_PATH,
        input_names=["input"], output_names=["yaw"],
        opset_version=13, dynamic_axes=None,
        dynamo=False,  # legacy exporter: clean opset-13 graph for TPU-MLIR
    )
    n_calib = make_calib_images()
    print(f"params           : {n_params:,}")
    print(f"output shape     : {tuple(out.shape)}")
    print(f"ONNX             : {ONNX_PATH}")
    print(f"calib images     : {n_calib} in {CALIB_DIR}")
    onnx_mb = os.path.getsize(ONNX_PATH) / 1e6
    print(f"ONNX size        : {onnx_mb:.3f} MB")


if __name__ == "__main__":
    main()
