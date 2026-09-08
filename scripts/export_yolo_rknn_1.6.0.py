#!/usr/bin/env python3
"""Exports a YOLO ONNX model to RKNN int8 for rv1106, using the raw
rknn-toolkit2 API directly (not Ultralytics' `format="rknn"` wrapper --
that wrapper targets rknn-toolkit2's CURRENT API/parameter names, which
this old 1.6.0 release predates).

**Why 1.6.0 specifically, not the current 2.3.2:** the real Luckfox
Pico Ultra's on-device runtime (confirmed via SSH, 2026-09-08) is
`librknnmrt version: 1.6.0` -- an older, RV1106-specific vendor build
from Luckfox's own SDK, itself already NEWER than what's on the public
airockchip/rknpu2 repo (master branch there is 1.5.2). A model exported
with a newer toolkit (2.3.2) embeds a newer model-format version the
1.6.0 runtime can't parse ("E RKNN: incompatible model version"). There
is no newer official RV1106 runtime to deploy instead -- 1.6.0 already
is the latest known build for this exact chip -- so the fix is
re-exporting with the MATCHING toolkit version, not upgrading the
device.

**Run this inside docker/convert.Dockerfile's container** (linux/amd64,
x86_64 emulation required -- 1.6.0 predates rknn-toolkit2's 2.3.0 aarch64
support, so it has no arm64 wheel, unlike the current 2.3.2 release which
runs natively on Apple Silicon).

Usage: python export_yolo_rknn_1.6.0.py <model.onnx> <target_platform> <dataset.txt> <output.rknn>
"""
import sys

from rknn.api import RKNN


def main():
    if len(sys.argv) != 5:
        print(f"usage: {sys.argv[0]} <model.onnx> <target_platform> <dataset.txt> <output.rknn>")
        sys.exit(1)
    onnx_path, target, dataset_txt, out_path = sys.argv[1:5]

    rknn = RKNN(verbose=False)
    print("=== config ===")
    ret = rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform=target)
    assert ret == 0, f"config failed: {ret}"

    print("=== load_onnx ===")
    ret = rknn.load_onnx(model=onnx_path)
    assert ret == 0, f"load_onnx failed: {ret}"

    print("=== build (int8 quantization) ===")
    ret = rknn.build(do_quantization=True, dataset=dataset_txt)
    assert ret == 0, f"build failed: {ret}"

    ret = rknn.export_rknn(out_path)
    print(f"export_rknn -> {out_path}, ret={ret}")
    rknn.release()


if __name__ == "__main__":
    main()
