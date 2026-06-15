#!/usr/bin/env bash
# Convert tiny_yaw_cnn.onnx -> INT8 .cvimodel for the SG2002 (cv181x) NPU.
#
# Runs the SOPHGO TPU-MLIR toolchain. Two ways to get that toolchain:
#   (a) pip install tpu_mlir   (x86 Linux; what we used -- no Docker)
#   (b) the sophgo/tpuc_dev Docker image, then run this same script inside it
#
# Output: tiny_yaw_cnn_int8.cvimodel  (deploy this to the board)
set -euo pipefail
cd "$(dirname "$0")"

NAME=tiny_yaw_cnn
CHIP=cv181x            # SG2002 belongs to the cv181x family
ONNX=${NAME}.onnx
MLIR=${NAME}.mlir
CALI=${NAME}_cali_table
OUT=${NAME}_int8.cvimodel

# Grayscale 0-255 -> 0-1 :  mean 0, scale 1/255
echo "== [1/3] model_transform =="
model_transform.py \
  --model_name "$NAME" \
  --model_def "$ONNX" \
  --input_shapes "[[1,1,128,128]]" \
  --mean 0.0 \
  --scale 0.00392156862745098 \
  --pixel_format gray \
  --mlir "$MLIR"

echo "== [2/3] run_calibration (60 frames) =="
run_calibration.py "$MLIR" \
  --dataset calib_images \
  --input_num 60 \
  -o "$CALI"

echo "== [3/3] model_deploy (INT8, $CHIP) =="
model_deploy.py \
  --mlir "$MLIR" \
  --quantize INT8 \
  --calibration_table "$CALI" \
  --chip "$CHIP" \
  --model "$OUT"

echo
echo "DONE -> $OUT"
ls -lh "$OUT"
