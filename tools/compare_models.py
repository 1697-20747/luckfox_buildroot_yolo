#!/usr/bin/env python3
"""Compare two RKNN YOLOv8n input sizes on the same frames, the way the board
feeds them, using rknn-toolkit2 1.6.0's simulator (int8, same calibration
recipe as tools/export_yolov8_rknn.sh).

  640x640: the 960x540 camera frame STRETCHED to 640x640 (what the server
           did before 2026-10-12)
  800x448: the frame resized to 800x448 (same proportions)

Reference ("pseudo ground truth"): full-precision YOLOv8n at 960 letterboxed,
computed separately (reference.json, tools/export_yolov8_rknn.sh's ONNX
container) because the RKNN container has no torch.

Scores each model by recall and precision against the reference at IoU 0.5,
for the server's allowed classes only (person, bicycle, car, motorcycle,
bus, truck).

Run inside luckfox-rknn-convert-160:
  python tools/compare_models.py build/model-compare
"""
import json
import sys

import cv2
import numpy as np
from rknn.api import RKNN

ALLOWED = [0, 1, 2, 3, 5, 7]
BOX_T, NMS_T = 0.25, 0.45
STRIDES = [8, 16, 32]


def dfl(x):  # x: (4*16,) -> 4 distances
    x = x.reshape(4, 16)
    e = np.exp(x - x.max(1, keepdims=True))
    return (e / e.sum(1, keepdims=True) * np.arange(16)).sum(1)


def decode(outs, in_w, in_h):
    boxes, scores, classes = [], [], []
    for s in range(3):
        box, score = outs[s * 3][0], outs[s * 3 + 1][0]  # (64,H,W), (80,H,W)
        _, gh, gw = box.shape
        for i in range(gh):
            for j in range(gw):
                sc = score[ALLOWED, i, j]
                k = int(np.argmax(sc))
                if sc[k] <= BOX_T:
                    continue
                d = dfl(box[:, i, j])
                st = STRIDES[s]
                x1, y1 = (j + 0.5 - d[0]) * st, (i + 0.5 - d[1]) * st
                x2, y2 = (j + 0.5 + d[2]) * st, (i + 0.5 + d[3]) * st
                boxes.append([x1, y1, x2, y2]); scores.append(float(sc[k])); classes.append(ALLOWED[k])
    keep = cv2.dnn.NMSBoxes([[b[0], b[1], b[2] - b[0], b[3] - b[1]] for b in boxes], scores, BOX_T, NMS_T)
    keep = np.array(keep).flatten().tolist() if len(keep) else []
    return [(boxes[k], scores[k], classes[k]) for k in keep]


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0


def match(dets, ref):
    used, tp = set(), 0
    for b, _, c in sorted(dets, key=lambda d: -d[1]):
        best, bi = 0, -1
        for k, (rb, rc) in enumerate(ref):
            if k in used or rc != c:
                continue
            v = iou(b, rb)
            if v > best:
                best, bi = v, k
        if best >= 0.5:
            used.add(bi); tp += 1
    return tp


def main():
    d = sys.argv[1]
    ref = json.load(open(f"{d}/reference.json"))  # {frame: [[x1,y1,x2,y2,cls], ...]} in 960x540 coords
    frames = sorted(ref)
    results = {}
    for name, (w, h) in {"640x640 stretched": (640, 640), "800x448": (800, 448)}.items():
        rk = RKNN(verbose=False)
        rk.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform="rv1106")
        rk.load_onnx(model=f"build/models/yolov8n-{h}x{w}.onnx")
        rk.build(do_quantization=True, dataset=f"build/models/calib-{h}x{w}/dataset.txt")
        rk.init_runtime()
        tp = n_det = n_ref = 0
        for f in frames:
            img = cv2.cvtColor(cv2.imread(f"{d}/frames/{f}"), cv2.COLOR_BGR2RGB)  # 960x540
            inp = cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)
            outs = rk.inference(inputs=[inp[None]], data_format="nhwc")
            dets = decode(outs, w, h)
            sx, sy = 960 / w, 540 / h
            dets = [([b[0] * sx, b[1] * sy, b[2] * sx, b[3] * sy], s, c) for b, s, c in dets]
            r = [(x[:4], x[4]) for x in ref[f]]
            tp += match(dets, r); n_det += len(dets); n_ref += len(r)
        rk.release()
        results[name] = {"recall": tp / max(n_ref, 1), "precision": tp / max(n_det, 1),
                         "detections": n_det, "reference_objects": n_ref, "matched": tp}
        print(name, results[name], flush=True)
    json.dump(results, open(f"{d}/compare.json", "w"), indent=1)


if __name__ == "__main__":
    main()
