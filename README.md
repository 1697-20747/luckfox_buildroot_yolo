# luckfox_buildroot_yolo

Real-time YOLOv8n object detection on a **Luckfox Pico Ultra** (Rockchip RV1106G3, 1 TOPS NPU), with
a live HUD video stream, navigation output and a web UI, all served from the board over USB. It runs
on Luckfox's Buildroot image and is built and deployed entirely from a Mac.

**Docs:**
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): how it works.
- [`CHANGELOG.md`](CHANGELOG.md): what changed, dated.
- [`PROJECT_MEMORY.md`](PROJECT_MEMORY.md): the full history.
- [`docs/vendor-report-emmc-hang-on-camera-restart.md`](docs/vendor-report-emmc-hang-on-camera-restart.md):
  the eMMC/camera hang investigation.

## Current state (2026-10-13)

| | |
|---|---|
| Live video with HUD | **30 fps** (camera rate), 960x540, hardware JPEG |
| Detection | **16 Hz** while streaming, **17 Hz** navigation-only |
| Model | YOLOv8n int8, 800x448 input (matches the 16:9 camera), RKNN 1.6.0 |
| CPU | ~73% average at 30 fps (colour conversion on the RGA, JPEG on the VENC) |
| Stability | 100/100 restarts clean, multiple 3 h runs with 0 resets; watchdog supervisor self-recovers |

## Quick start

1. Plug the board into the Mac by USB (CDC-NCM network gadget). On a new Mac, once:
   ```bash
   networksetup -setmanual "rk3xxx" 172.32.0.1 255.255.0.0
   ```
2. Open **http://172.32.0.70:8080/**. Nothing to install.
3. SSH (key only): `ssh root@172.32.0.70`.

The web UI offers:
- HUD or plain view, and one viewer at a time (a second viewer takes over);
- a per-session object list (latest 20, a full-list page, CSV export, reset);
- board stats, and WiFi on/off (off by default);
- accelerometer tilt with **Set level** and **6-position calibration**;
- the on-board docs site at `/docs/`.

The board takes its clock from the browser on page load.

### HTTP API

| Endpoint | |
|---|---|
| `/stream` | MJPEG video (one viewer; takeover) |
| `/api/status` | fps, detection rate, timings, CPU, memory, temperature, tilt |
| `/api/detections`, `/api/detections/stream` | navigation output (JSON / SSE): class, score, box, centre, bearing, pitch/roll |
| `/api/objects?limit=N`, `/api/objects/reset` | session object log |
| `/api/view?mode=hud\|plain`, `/api/wifi`, `/api/time`, `/api/level`, `/api/accelcal` | controls |

## What runs on the board

```
camera → ISP (3A) → yolo_camd ──DMA-BUF──► stream server ──► HTTP :8080
                    (stream started once,   RGA convert → NPU (YOLOv8n) → HUD → VENC JPEG
                     never stopped)
yolo_supervisor: owns the hardware watchdog, resets the board on a hang, keeps the crash log
```

Started at boot by `/usr/bin/yolo-app-start` (from `buildroot/overlay-yolo/`). Settings live in
`/root/rknn_test/app.env` (source: [`scripts/app.env`](scripts/app.env)):

| Setting | Default | |
|---|---|---|
| `LUCKFOX_MODEL` | 800x448 model | remove to use the 640x640 model |
| `LUCKFOX_PIPELINE=hwjpeg`, `LUCKFOX_HW_JPEG=1` | on | hardware JPEG; remove for libjpeg-turbo |
| `LUCKFOX_RGA` | on | `0`: colour conversion on the CPU |
| `LUCKFOX_DECOUPLED` | on | `0`: video paced by detection (16 fps) |
| `LUCKFOX_CAMD` | on | `0`: server captures directly (not restart-safe) |

## Build and deploy (on the Mac)

The Luckfox SDK lives in an amd64 Ubuntu Docker image (`docker/sdk.Dockerfile`), with the SDK in the
Docker volume `luckfox-sdk`.

```bash
bash tools/build_app.sh     # cross-compile server, camd, supervisor; stage the model and web UI
bash tools/deploy_app.sh    # install on the board (stage + rename), restart the server
```

- **Firmware:** `buildroot/apply_overlay.sh` and `apply_kernel_patches.sh`, then `./build.sh` in the
  SDK. Flash with `tools/flash_buildroot.sh`. Back up and restore with `tools/backup_emmc.sh` /
  `restore_emmc.sh`.
- **Models:** `bash tools/export_yolov8_rknn.sh 448 800` exports YOLOv8n at any input size:
  Rockchip's YOLOv8 fork, then ONNX, then rknn-toolkit2 1.6.0 int8. `tools/compare_models.py`
  compares two sizes' accuracy in the simulator.
- **Testing:** `tools/restart_stress.sh` (restart cycles, saves crash logs), `tools/continuous_soak.sh`
  (multi-hour runs), `scripts/rga_bench.c` (RGA vs CPU).

### Why rknn-toolkit2 1.6.0

The board's runtime (`librknnmrt`) is 1.6.0, and models from newer toolkits (2.x) won't load. No
newer RV1106 runtime exists, so models are exported with the matching toolkit
(`docker/convert.Dockerfile`).

## Hardware notes

- **Camera:** SC3336 on the CSI connector, captured from `rkisp_mainpath` (`/dev/video11`) as NV12
  960x540.
- **Accelerometer:** MPU-6050 on I2C2, polled through IIO (kernel patch
  `buildroot/kernel-patches/0001`). It gives pitch/roll only, with no heading.

| Header pin | Function |
|---|---|
| 31 | 3.3 V |
| 32 | I2C2 SCL (GPIO1_A0) |
| 33 | I2C2 SDA (GPIO1_B0) |
| 40 | GPIO1_A1, wired to MPU-6050 INT (unused: the driver runs polled) |
| 41 | GND |

- **Enclosure:** STLs and renders in `drawings/LuckFox BOX/`. `tools/render_assembly.py` renders
  the assembly.

## Layout

| Path | |
|---|---|
| `scripts/` | board software: `luckfox_video_hud_stream_cam.c` (stream server), `yolo_camd.c`, `yolo_supervisor.c`, `www/` (web UI), `app.env` |
| `buildroot/` | overlay (init scripts, `yolo-app-start`, `yolo-wifi`, `yolo-healthlog`), kernel patches, SDK helper scripts |
| `tools/` | build, deploy, flash, model export, test and render scripts |
| `docker/` | SDK and model-conversion images |
| `docs/` | architecture summary, vendor report |
| `drawings/` | enclosure CAD |
| `vendor_reference/` | Rockchip `rknn_model_zoo` reference code, kept for comparison |
| `board/`, `scripts/luckfox_video_hud_stream.c`, `luckfox_yolov8_detect.c`, `luckfox_rknn_test.c` | Ubuntu-era (pre-2026-10-10) files, kept for history |
