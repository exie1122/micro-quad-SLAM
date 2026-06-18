import unittest

import numpy as np

from preprocess import resize_gray
from train import PairDataset, split_train_val


class PreprocessTests(unittest.TestCase):
    def test_all_resize_modes_are_square(self):
        image = np.arange(48, dtype=np.uint8).reshape(6, 8)
        for mode in ("stretch", "center_crop", "letterbox"):
            self.assertEqual(resize_gray(image, 4, mode).shape, (4, 4))

    def test_center_crop_removes_equal_sides(self):
        image = np.zeros((4, 8), dtype=np.uint8)
        image[:, 2:6] = 255
        out = resize_gray(image, 4, "center_crop")
        self.assertTrue(np.all(out == 255))


class SplitTests(unittest.TestCase):
    def test_train_validation_boundary_has_no_shared_frames(self):
        stride = 4
        pairs = [
            {"img_a": f"f{i}", "img_b": f"f{i + stride}", "rel_yaw": 0.0}
            for i in range(100)
        ]
        manifest = {
            "frame_stride": stride,
            "splits": {"train": ["seq"]},
            "sequences": {"seq": pairs},
        }
        train, val = split_train_val(manifest, 0.2)
        train_frames = {p[k] for p in train for k in ("img_a", "img_b")}
        val_frames = {p[k] for p in val for k in ("img_a", "img_b")}
        self.assertFalse(train_frames & val_frames)

    def test_zero_motion_augmentation_sets_zero_label(self):
        ds = PairDataset([], 8, {}, augment={"enabled": True, "zero_motion_p": 1.0})
        a = np.arange(64, dtype=np.uint8).reshape(8, 8)
        _, b, angle = ds._augment(a, np.flipud(a), 0.3)
        self.assertEqual(angle, 0.0)
        self.assertTrue(np.allclose(b, a.astype(np.float32) / 255.0))


if __name__ == "__main__":
    unittest.main()
