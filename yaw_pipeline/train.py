"""
Stage 2 - train.

Trains YawNet on the TRAIN sequences (validation = contiguous temporal tail of
each train sequence, so no shuffled frame-level leakage). Metric = mean absolute
yaw error in degrees. Saves the best checkpoint and exports an FP32 ONNX (opset 13).
"""
import os
import json
import argparse
import numpy as np
import cv2
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torch.utils.data import WeightedRandomSampler

from config_util import load_config, ensure_workdir
from model import YawNet, predict_angle
from preprocess import load_gray


class PairDataset(Dataset):
    """Frame pairs -> (input[2,H,W] float32 in [0,1], target[sin,cos])."""
    def __init__(self, pairs, img_size, frame_cache, resize_mode="stretch", augment=None):
        self.pairs = pairs
        self.size = img_size
        self.cache = frame_cache
        self.resize_mode = resize_mode
        self.augment = augment or {}

    def _img(self, path):
        if path not in self.cache:
            self.cache[path] = load_gray(path, self.size, self.resize_mode)
        return self.cache[path]

    def _augment(self, a, b, angle):
        cfg = self.augment
        if not cfg.get("enabled", False):
            return a, b, angle

        if np.random.random() < cfg.get("temporal_swap_p", 0.0):
            a, b, angle = b, a, -angle
        if np.random.random() < cfg.get("horizontal_flip_p", 0.0):
            a, b, angle = np.fliplr(a), np.fliplr(b), -angle
        # Identical-frame examples anchor the estimator at zero and prevent a
        # small per-pair bias from becoming catastrophic accumulated drift.
        if np.random.random() < cfg.get("zero_motion_p", 0.0):
            b, angle = a.copy(), 0.0

        brightness = float(cfg.get("brightness", 0.0))
        contrast = float(cfg.get("contrast", 0.0))
        noise_std = float(cfg.get("noise_std", 0.0))
        blur_p = float(cfg.get("blur_p", 0.0))

        def photo(im):
            out = im.astype(np.float32) / 255.0
            if contrast:
                out = (out - 0.5) * np.random.uniform(1 - contrast, 1 + contrast) + 0.5
            if brightness:
                out += np.random.uniform(-brightness, brightness)
            if noise_std:
                out += np.random.normal(0.0, noise_std, out.shape).astype(np.float32)
            out = np.clip(out, 0.0, 1.0)
            if np.random.random() < blur_p:
                out = cv2.GaussianBlur(out, (3, 3), np.random.uniform(0.2, 1.2))
            return out.astype(np.float32, copy=False)

        return photo(a), photo(b), angle

    def __len__(self):
        return len(self.pairs)

    def __getitem__(self, i):
        p = self.pairs[i]
        a, b, angle = self._augment(self._img(p["img_a"]), self._img(p["img_b"]),
                                    float(p["rel_yaw"]))
        if a.dtype != np.float32:
            a = a.astype(np.float32) / 255.0
            b = b.astype(np.float32) / 255.0
        x = np.stack([a, b], 0)
        y = np.array([np.sin(angle), np.cos(angle)], dtype=np.float32)
        return torch.from_numpy(x), torch.from_numpy(y)


def mae_deg(pred_sincos, gt_sincos):
    pa = predict_angle(pred_sincos)
    ga = predict_angle(gt_sincos)
    d = np.arctan2(np.sin(pa - ga), np.cos(pa - ga))
    return float(np.degrees(np.abs(d)).mean())


def split_train_val(manifest, val_tail_frac):
    train, val = [], []
    purge = int(manifest.get("frame_stride", 1))
    for name in manifest["splits"]["train"]:
        pairs = manifest["sequences"][name]
        k = int(len(pairs) * (1 - val_tail_frac))
        # A pair at index i includes frame i+stride. Purge the boundary so no
        # underlying image appears in both training and validation.
        train += pairs[:max(0, k - purge)]
        val += pairs[k:]            # contiguous tail
    return train, val


def evaluate(model, loader, device):
    model.train(False)
    preds, gts = [], []
    with torch.no_grad():
        for x, y in loader:
            out = model(x.to(device)).cpu().numpy()
            preds.append(out); gts.append(y.numpy())
    preds, gts = np.concatenate(preds), np.concatenate(gts)
    # normalize predicted vectors before angle conversion
    preds = preds / (np.linalg.norm(preds, axis=1, keepdims=True) + 1e-9)
    return mae_deg(preds, gts)


def export_onnx(model, path, img_size):
    model.train(False)
    dummy = torch.randn(1, 2, img_size, img_size)
    torch.onnx.export(model.cpu(), dummy, path, input_names=["input"],
                      output_names=["yaw"], opset_version=13, dynamo=False)


def yaw_balancing_sampler(pairs, bins_deg):
    """Inverse-frequency sampling by absolute yaw bin."""
    angle = np.abs(np.degrees([p["rel_yaw"] for p in pairs]))
    bin_id = np.digitize(angle, np.asarray(bins_deg, dtype=np.float64))
    counts = np.bincount(bin_id, minlength=len(bins_deg) + 1)
    weights = np.array([1.0 / max(counts[i], 1) for i in bin_id], dtype=np.float64)
    return WeightedRandomSampler(torch.as_tensor(weights, dtype=torch.double),
                                 num_samples=len(weights), replacement=True), counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)
    tc = cfg["train"]

    torch.manual_seed(tc["seed"]); np.random.seed(tc["seed"])
    device = tc["device"] if (tc["device"] == "cpu" or torch.cuda.is_available()) else "cpu"
    print(f"[train] device={device}")

    manifest = json.load(open(os.path.join(workdir, "manifest.json")))
    train_pairs, val_pairs = split_train_val(manifest, tc["val_tail_frac"])
    print(f"[train] train pairs={len(train_pairs)}  val pairs={len(val_pairs)}")

    cache = {}
    size = manifest["img_size"]
    resize_mode = cfg["dataset"].get("resize_mode", "stretch")
    sampler = None
    balance_counts = None
    if tc.get("balance_yaw", False):
        bins = tc.get("balance_bins_deg", [0.5, 1.0, 2.0, 4.0, 8.0])
        sampler, balance_counts = yaw_balancing_sampler(train_pairs, bins)
        print(f"[train] yaw balancing bins(deg)={bins} raw counts={balance_counts.tolist()}")
    train_ld = DataLoader(
        PairDataset(train_pairs, size, cache, resize_mode, tc.get("augment")),
        batch_size=tc["batch_size"], shuffle=sampler is None, sampler=sampler, num_workers=0)
    val_ld = DataLoader(PairDataset(val_pairs, size, cache, resize_mode), batch_size=tc["batch_size"],
                        shuffle=False, num_workers=0)

    # naive zero-predictor baseline (always predicts 0 yaw): MAE = mean|gt yaw|
    val_gt = np.array([[np.sin(p["rel_yaw"]), np.cos(p["rel_yaw"])] for p in val_pairs])
    zero_mae = mae_deg(np.tile([0.0, 1.0], (len(val_gt), 1)), val_gt)
    print(f"[train] zero-predictor val MAE = {zero_mae:.2f} deg (model must beat this)")

    model = YawNet().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=tc["lr"], weight_decay=tc["weight_decay"])
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=tc["epochs"])

    best = (1e9, None)
    ckpt = os.path.join(workdir, "yaw_net.pt")
    for ep in range(tc["epochs"]):
        model.train(True)
        for x, y in train_ld:
            x, y = x.to(device), y.to(device)
            out = model(x)
            out = out / (out.norm(dim=1, keepdim=True) + 1e-9)   # unit (sin,cos)
            # cosine-distance loss = 1 - cos(pred_angle - gt_angle): directly
            # minimizes angular error, unbiased by the tiny-angle magnitude.
            loss = (1.0 - (out * y).sum(dim=1)).mean()
            opt.zero_grad(); loss.backward(); opt.step()
        sched.step()
        vmae = evaluate(model, val_ld, device)
        if vmae < best[0]:
            best = (vmae, ep)
            torch.save(model.state_dict(), ckpt)
        if ep % 5 == 0 or ep == tc["epochs"] - 1:
            print(f"  epoch {ep:3d}  val MAE = {vmae:6.2f} deg   (best {best[0]:.2f} @ {best[1]})")

    print(f"[train] best val MAE = {best[0]:.2f} deg  -> {ckpt}")
    if best[0] >= zero_mae:
        print(f"[train] WARNING: model did not beat the zero-yaw baseline "
              f"({best[0]:.2f} >= {zero_mae:.2f} deg)")
    model.load_state_dict(torch.load(ckpt, map_location="cpu"))
    onnx_path = os.path.join(workdir, "yaw_net.onnx")
    export_onnx(model, onnx_path, size)
    json.dump({"best_val_mae_deg": best[0], "best_epoch": best[1],
               "zero_val_mae_deg": zero_mae, "img_size": size,
               "resize_mode": resize_mode, "yaw_balance_counts":
               balance_counts.tolist() if balance_counts is not None else None,
               "beats_zero_baseline": bool(best[0] < zero_mae)},
              open(os.path.join(workdir, "train_metrics.json"), "w"))
    print(f"[train] exported FP32 ONNX -> {onnx_path}")


if __name__ == "__main__":
    main()
