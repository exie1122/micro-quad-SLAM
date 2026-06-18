"""
YawNet: NPU-friendly relative-yaw regressor.

Input : [B, 2, 128, 128] grayscale frame PAIR (channel 0 = frame a, 1 = frame b),
        pixel values scaled to [0, 1].
Output: [B, 2] = (sin, cos) of the relative yaw, each bounded to [-1, 1] via tanh.

We regress (sin, cos) instead of the raw angle to avoid the +/-180 deg wraparound
discontinuity. The output is L2-normalized in post-processing (not in the graph)
and converted to an angle with atan2.

Backbone uses only standard conv + BN + ReLU + global average pool + one FC, all
INT8-stable and supported by TPU-MLIR. BN folds into the preceding conv at export.
"""
import torch
import torch.nn as nn


class YawNet(nn.Module):
    def __init__(self):
        super().__init__()
        def block(ci, co):
            return nn.Sequential(
                nn.Conv2d(ci, co, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(co),
                nn.ReLU(inplace=True),
            )
        self.features = nn.Sequential(
            block(2, 16),    # 128 -> 64
            block(16, 32),   # 64 -> 32
            block(32, 48),   # 32 -> 16
            block(48, 64),   # 16 -> 8
            block(64, 64),   # 8 -> 4
            nn.AdaptiveAvgPool2d(1),
        )
        self.head = nn.Linear(64, 2)

    def forward(self, x):
        x = self.features(x)
        x = torch.flatten(x, 1)
        return torch.tanh(self.head(x))   # bounded (sin, cos) in [-1, 1]


def predict_angle(sincos):
    """[*,2] (sin,cos) -> angle in radians, after L2 normalization."""
    import numpy as np
    s, c = sincos[..., 0], sincos[..., 1]
    return np.arctan2(s, c)
