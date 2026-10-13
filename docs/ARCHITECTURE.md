# Software install and architecture (short version)

This is the state as of 2026-10-12 (evening). `CHANGELOG.md` is the dated record, and `PROJECT_MEMORY.md` has
the full story.

## Board and image

| | |
|---|---|
| Board | Luckfox Pico Ultra: RV1106G3, one Cortex-A7 core, 1 TOPS NPU, 256 MB RAM, 7.1 GB eMMC, no SD slot |
| Camera | SC3336 on MIPI CSI, captured from `rkisp_mainpath` (`/dev/video11`) as NV12 960x540 |
| OS | Luckfox SDK Buildroot (`BoardConfig-EMMC-Buildroot-RV1106_Luckfox_Pico_Ultra-IPC.mk`), kernel 5.10.160 plus `buildroot/kernel-patches/` (MPU-6050 on i2c2, watchdog + ramoops, SysRq), root mounted `rw` |
| Access | USB CDC-NCM gadget: board `172.32.0.70`, Mac `172.32.0.1`. SSH key-only as root. WiFi off by default (web UI button) |

## Build and install (all from a Mac)

- **SDK:** runs in an amd64 Ubuntu 22.04 Docker image, with the SDK in Docker volume `luckfox-sdk`
  (case-sensitive filesystem).
- **Firmware:** `buildroot/apply_overlay.sh` + `apply_kernel_patches.sh`, then `./build.sh`. Flash
  with `rkdeveloptool` (maskrom), or for the kernel alone, `dd` the boot image to `/dev/mmcblk0p4`.
- **App:** `tools/build_app.sh` cross-compiles everything below into `build/app_src/`.
  `tools/deploy_app.sh` installs it in `/root/rknn_test` by staging and renaming, never overwriting a
  running binary.
- **Overlay:** `buildroot/overlay-yolo/` holds the init scripts, `yolo-app-start`, `yolo-wifi` and
  `yolo-healthlog`.

## What runs, in boot order

```
S05watchdog   yolo_supervisor (static, mlocked) owns /dev/watchdog; yolo-healthlog
S21appinit    Rockchip's RkLunch.sh, with rkipc replaced by /usr/bin/yolo-app-start:
                rkaiq_3A_server       auto exposure / white balance (sensor IQ file)
                yolo_camd             opens the camera stream ONCE, never stops it
                luckfox_video_hud_stream_hwjpeg   (restart loop; _cam = software JPEG)
```

```
 camera ─ ISP ─ /dev/video11 ──► yolo_camd ──(DMA-BUF fds over \0yolo-camd)──► stream server
                    ▲                                                            │
            rkaiq_3A_server     RGA: NV12→RGB 960x540 (display) + NV12→RGB 800x448 (model)
                                NEON int8 → NPU (YOLOv8n, RKNN 1.6) in a worker thread
                                viewer: newest frame every loop + latest boxes (30 fps / 15 Hz)
                                nav only: pipelined capture | NPU | output (17 Hz)
                                HUD → RGA RGB→NV12 → hardware JPEG
                                                                                 │
                                    HTTP :8080  /  /stream (MJPEG, one viewer)  /api/status
                                                /api/detections(/stream)  /api/objects  /api/wifi
                                                /docs/ (Hugo mini site, board only)
```

### Components

- **`yolo_camd`** (`scripts/yolo_camd.c`)
  - Owns the camera stream for the whole boot and never stops it.
  - Exports its 4 capture buffers as DMA-BUFs. The server borrows the newest buffer per frame, with
    no copy.
  - This is the stability fix: every eMMC hang on this board followed a camera stream *restart* while
    3A was running (see `docs/vendor-report-emmc-hang-on-camera-restart.md`). Now restarting the
    server never restarts the stream.
  - `LUCKFOX_CAMD=0` captures directly instead.
- **Stream server** (`scripts/luckfox_video_hud_stream_cam.c`)
  - Detection on the NPU in a worker thread; HUD with live tilt from the MPU-6050.
  - JPEG through Rockit VENC (hardware, default binary `..._hwjpeg`), or static libjpeg-turbo.
  - Navigation output: object centres, bearings and an SSE stream.
  - Web UI and object log.
  - Runs only while someone is watching or subscribed.
- **`yolo_supervisor`** (`scripts/yolo_supervisor.c`)
  - Kicks the hardware watchdog only while the server's heartbeat is fresh and a `/userdata`
    write + fsync completes.
  - On a fault it dumps task stacks and the eMMC controller registers into ramoops, then lets the
    watchdog reset the board (recovery about 45 s).
  - Gives up after 3 consecutive faults, so it can't cause a reset loop.
- **`yolo-app-start`**
  - Boot-loop guard: if the previous boot didn't clear its marker, the next boot runs with
    experimental options off.
  - Settings come from `/root/rknn_test/app.env`.

## Performance and stability (2026-10-12)

| | |
|---|---|
| Model | YOLOv8n, 800x448 input (16:9, matches the camera), int8; 640x640 as fallback |
| Video with HUD | 30 fps (camera rate), detection 14.9 Hz alongside |
| Navigation only | 17.1 Hz |
| Per frame | NPU 58 ms; RGA conversions ~1 ms each; quantise 2 ms (NEON); decode 1.8 ms; HUD 4–5 ms; JPEG 3 ms |
| CPU | ~73% average while streaming 30 fps (was 86% at 16 fps before the RGA) |
| Accuracy vs 640x640 | recall 84% vs 61%, precision 92% vs 94% (`tools/compare_models.py`) |
| Memory | about 126 MB free, flat |
| Restart test | 100/100 clean with `yolo_camd`; 50/50 with hardware JPEG; 30/30 each on the 800x448, RGA and decoupled builds |
| Continuous run | 3 h each: software JPEG, hardware JPEG, and RGA + decoupled (30.0 fps / 14.9 Hz throughout): 0 resets, memory flat |

## Tools

| Tool | Purpose |
|---|---|
| `tools/restart_stress.sh` | Server restart cycles; saves crash logs after any self-reset |
| `tools/continuous_soak.sh` | Multi-hour run with per-minute stats |
| `tools/export_yolov8_rknn.sh H W` | Reproducible YOLOv8n → RKNN export at any input size |
| `tools/compare_models.py` | Simulator accuracy comparison of two input sizes |
| `tools/backup_emmc.sh`, `restore_emmc.sh`, `flash_buildroot.sh` | Whole-board backup and restore |
