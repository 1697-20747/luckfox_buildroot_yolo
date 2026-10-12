#!/bin/bash
# Export YOLOv8n for the RV1106 NPU at a chosen input size, end to end, in
# Docker on the Mac:
#   1. ONNX with Rockchip's RKNN-optimised head (airockchip/ultralytics_yolov8,
#      format="rknn" writes an ONNX with the DFL head restructured into the
#      9-output form the server's post-processing expects). torch is pinned to
#      a pre-dynamo version so the export stays a single-file opset-12 ONNX.
#   2. Calibration images: frames from test_videos/bikes.mp4 + bus.jpg,
#      letterboxed to the model size (int8 quantisation needs inputs shaped
#      like the real ones).
#   3. RKNN int8 for rv1106 with rknn-toolkit2 1.6.0 (the board's runtime
#      version; scripts/export_yolo_rknn_1.6.0.py, image luckfox-rknn-convert-160).
#
#   bash tools/export_yolov8_rknn.sh 448 800
#   -> build/models/yolov8n-448x800-rv1106-1.6.0.rknn
set -euo pipefail
H=${1:?height}; W=${2:?width}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/build/models"
mkdir -p "$OUT/calib-${H}x${W}"
NAME=yolov8n-${H}x${W}

# 1. ONNX (arm64-native container; only the RKNN conversion needs amd64)
if [ ! -f "$OUT/$NAME.onnx" ]; then
docker run --rm -v "$OUT":/out python:3.10-slim-bookworm bash -c "
set -e
apt-get update -qq && apt-get install -y -qq git libgl1 libglib2.0-0 > /dev/null
pip install -q --no-input torch==2.2.2 torchvision==0.17.2 --index-url https://download.pytorch.org/whl/cpu --extra-index-url https://pypi.org/simple
pip install -q --no-input 'numpy<2' 'onnx==1.14.1' opencv-python-headless
git clone -q --depth 1 https://github.com/airockchip/ultralytics_yolov8 /yolo
cd /yolo && pip install -q --no-input -e .
cd /tmp && python -c \"
from ultralytics import YOLO
YOLO('yolov8n.pt').export(format='rknn', imgsz=($H, $W), opset=12)
\"
f=\$(ls /tmp/yolov8n*.onnx | head -1); echo exported \$f; cp \$f /out/$NAME.onnx
"
fi
python3 -c "
import onnx; m=onnx.load('$OUT/$NAME.onnx')
print('opset', m.opset_import[0].version, 'input', [d.dim_value for d in m.graph.input[0].type.tensor_type.shape.dim], 'outputs', len(m.graph.output))
" 2>/dev/null || echo "(onnx not installed on the Mac; checked in step 3)"

# 2. Calibration set, letterboxed to WxH
docker run --rm --platform linux/amd64 -v "$ROOT":/work -w /work luckfox-rknn-convert-160 python -c "
import cv2, glob
H, W = $H, $W
out = 'build/models/calib-${H}x${W}'
def letterbox(img):
    h, w = img.shape[:2]; s = min(W / w, H / h)
    r = cv2.resize(img, (round(w * s), round(h * s)))
    c = cv2.copyMakeBorder(r, 0, 0, 0, 0, cv2.BORDER_CONSTANT)
    canvas = cv2.copyMakeBorder(r, (H - r.shape[0]) // 2, H - r.shape[0] - (H - r.shape[0]) // 2,
                                (W - r.shape[1]) // 2, W - r.shape[1] - (W - r.shape[1]) // 2,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return canvas
paths = []
cap = cv2.VideoCapture('test_videos/bikes.mp4'); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
for i in range(30):
    cap.set(cv2.CAP_PROP_POS_FRAMES, i * max(1, n // 30)); ok, f = cap.read()
    if ok:
        p = f'{out}/bikes{i:02d}.jpg'; cv2.imwrite(p, letterbox(f)); paths.append(p)
p = f'{out}/bus.jpg'; cv2.imwrite(p, letterbox(cv2.imread('vendor_reference/bus.jpg'))); paths.append(p)
open(f'{out}/dataset.txt', 'w').write('\n'.join('/work/' + q for q in paths) + '\n')
print(len(paths), 'calibration images', W, 'x', H)
"

# 3. RKNN int8, toolkit 1.6.0
docker run --rm --platform linux/amd64 -v "$ROOT":/work -w /work luckfox-rknn-convert-160 bash -c "
python -c 'import rknn.api, pkg_resources; print(\"toolkit\", pkg_resources.get_distribution(\"rknn-toolkit2\").version)'
python scripts/export_yolo_rknn_1.6.0.py build/models/$NAME.onnx rv1106 build/models/calib-${H}x${W}/dataset.txt build/models/$NAME-rv1106-1.6.0.rknn
"
ls -l "$OUT/$NAME-rv1106-1.6.0.rknn"
