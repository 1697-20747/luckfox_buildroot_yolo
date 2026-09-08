# luckfox_buildroot_yolo

Investigating whether the Luckfox Pico Ultra (Rockchip RV1106G3, ~1 TOPS NPU) can run YOLO26 via
RKNN — a possible offload target for detection/positional processing that's been too slow on plain
Raspberry Pi 5 CPU. See `PROJECT_MEMORY.md` for the full real, dated history of what's actually
been tried and found — this README is just the current entry point.

**Status as of 2026-09-08: yolov8n detection is fully working end-to-end on real hardware; YOLO26
itself remains blocked.** YOLO26 fails to even load on this board's NPU (an unsupported `MatMul` op
in its attention block) — likely an architectural limitation of this specific chip, corroborated by
Rockchip's own official support lists (RV1103/RV1106 officially supports only YOLOv5 among the YOLO
family; YOLOv8/v10/11 are all absent). But `yolov8n`, using Rockchip's own RKNN-optimized export,
now produces **real, correct detections** on the actual NPU — tested against Rockchip's own
reference `bus.jpg`: 3 people correctly localized, the vehicle correctly boxed (classified `truck`
instead of `bus`, a plausible minor int8-quantization mix-up between similar classes, not a
pipeline bug). Plan going forward, per 1697-20747: get this fully solid on YOLOv8 first (real video
processing next, live camera later), then port the same approach to YOLO26 once/if the attention-
block blocker is addressed. See `PROJECT_MEMORY.md` for the full details.

## Live HUD video stream over WiFi

`scripts/luckfox_video_hud_stream.c` runs a persistent detection pipeline against a video file and
serves it as a live MJPEG stream with an always-visible HUD overlay (boxes, labels, fps/inference-
time readout — apex_hud-inspired, not a literal port). Display resolution is decoupled from
inference resolution: the NPU always sees a 640x640 downscaled copy (inference cost stays
constant), while the stream/HUD renders at the source video's own native resolution by default —
real, measured tradeoff: full native res looks better but costs real fps (JPEG-encode+network-send
scale with pixel count). See `PROJECT_MEMORY.md` for the full profiled breakdown at both
resolutions; 30fps is not achievable at either.

```
sudo ./luckfox_video_hud_stream <model.rknn> <video.mp4> [labels.txt] [port] [display_w] [display_h]
```
`display_w`/`display_h` default to the source video's own resolution (auto-probed via `ffprobe`) —
pass them explicitly (e.g. `960 540`) to trade quality for fps. Then open
`http://<board-ip>:<port>/` in a browser on the same WiFi network.

Also reads the MPU-6050 accelerometer via Linux's standard IIO sysfs interface, auto-discovering
the right `iio:deviceN`. **Working end-to-end as of 2026-09-09** on a custom-built kernel (see
`PROJECT_MEMORY.md`) — confirmed live: `RESULT: ACCEL_FOUND /sys/bus/iio/devices/iio:device1
scale=0.000598`, real x/y/z readings, real measured sample rate **~3.6 Hz**, tied 1:1 to the video
loop's own ~3.5 fps steady-state (one `accel_update()` per rendered frame, not an independent
sensor-bus rate — a real IRQ line on pin 40 would decouple these, see below). Real readings pass
through an exponential-moving-average filter (`ACCEL_EMA_ALPHA` in the source) before being drawn,
to keep the HUD readout from jumping around on raw sensor noise. Draws "ACCEL: not connected"
(grey) as a fallback if no sensor is found, rather than omitting the panel or guessing a value.

## GPIO header pinout (confirmed real, not from an unreadable diagram)

Read directly off a real photo of the board's own pinout diagram (Waveshare-style), for the pins
used by the MPU-6050 accelerometer:

| Physical pin | Function | Notes |
|---|---|---|
| 31 | 3.3V | power |
| 32 | GPIO1_A0, alt-function `I2C2_SCL_M0` | SCL |
| 33 | GPIO1_B0, alt-function `I2C2_SDA_M0` | SDA |
| 40 | GPIO1_A1 | MPU-6050 **INT** (data-ready IRQ) — optional, see below |
| 41 | GND | confirmed ground (pin 40 is not, despite being adjacent) |

This board has three separate I2C buses in active use, not one shared bus — worth remembering
before wiring anything else up: **I2C2** (this GPIO header, used above for the MPU-6050), **I2C3**
(the LCD connector's touchscreen), **I2C4** (the CSI camera connector's sensor control — shares
pins with the CSI camera, so CSI and I2C4 can't both be configured at once by default).

Pin 40 (`GPIO1_A1`) is wired to the MPU-6050's `INT` pin for real interrupt-driven data-ready
signaling. It's optional, not load-bearing: the mainline `inv_mpu6050` driver hard-requires a real
IRQ line to set up its triggered-buffer/FIFO mode (`devm_request_irq()` on IRQ 0 fails with
`-EINVAL` otherwise), so without `INT` wired, the kernel driver patch in this repo's
`PROJECT_MEMORY.md` skips that setup and falls back to plain polled sysfs reads
(`in_accel_*_raw`) — which is all `luckfox_video_hud_stream.c` actually uses. Wiring `INT` unlocks
lower-latency/jitter interrupt-driven sampling as a future improvement, not a requirement to get
real readings.

## Live camera

Not connected yet. Real compatibility gap found before physically trying: this board's vendor image
ships only one camera sensor driver (`sc3336.ko`) — no IMX219/IMX708/OV5647 (Raspberry Pi Camera
Module) driver exists on it. A Pi Camera on the ribbon connector may not be electrically compatible
either, separate from the driver question. See `PROJECT_MEMORY.md` for the plan once a module is
physically connected (check `dmesg`/I2C detection first, before investing in kernel driver work).

## Try it yourself

```
scp scripts/luckfox_yolov8_detect.c vendor_reference/3rdparty/stb_image/stb_image.h \
    vendor_reference/coco_80_labels_list.txt <model>.rknn <image>.jpg \
    pico@<board-ip>:~/rknn_test/
ssh pico@<board-ip>
cd ~/rknn_test
mkdir -p 3rdparty/stb_image && mv stb_image.h 3rdparty/stb_image/
g++ -O2 -o luckfox_yolov8_detect luckfox_yolov8_detect.c -I. -I3rdparty/stb_image -lrknnmrt -lm
sudo ./luckfox_yolov8_detect <model>.rknn <image>.jpg coco_80_labels_list.txt
```

`vendor_reference/` holds real reference material fetched from `airockchip/rknn_model_zoo` (their
own `postprocess.cc`/`main.cc`/`yolov8_rv1106_1103.cc`, `utils/`, `stb_image`, `bus.jpg`,
`coco_80_labels_list.txt`) — kept for comparison/attribution, not built directly; `luckfox_yolov8_detect.c`
is this project's own leaner adaptation of the decode math from it.

## Layout

- `PROJECT_MEMORY.md` — dated real-session history, the actual source of truth.
- `LUCKFOX_ACCESS.local.md` — gitignored SSH credentials for the board (DHCP IP changes each
  session — rediscover via `arp -a` or the router's device list).
- `docker/convert.Dockerfile` — the real, working Docker image for running `rknn-toolkit2 1.6.0`
  (the version matching this board's actual on-device runtime) under x86_64 emulation on Apple
  Silicon. Build/run commands are in the Dockerfile's own header comment.
- `scripts/export_yolo_rknn_1.6.0.py` — runs inside that container: ONNX → RKNN int8 for `rv1106`.
- `scripts/clear_execstack.py` — works around a real QEMU-emulation bug (old `.so` files with an
  executable-stack flag fail to load under Rosetta/QEMU x86_64 emulation on Apple Silicon).
- `scripts/luckfox_rknn_test.c` — minimal real on-device inference test, using the zero-copy DMA
  memory API RV1106 actually requires (`rknn_create_mem`/`rknn_set_io_mem`, not the standard
  `rknn_inputs_set`/`rknn_outputs_get` most RKNN docs/demos default to). Compiles natively on-device
  (`gcc` is present on the board's Ubuntu 22.04 image — no cross-toolchain needed):
  ```
  gcc -o luckfox_rknn_test luckfox_rknn_test.c -I. -L/usr/lib -lrknnmrt -Wl,-rpath,/usr/lib
  sudo ./luckfox_rknn_test <model.rknn>
  ```

## Why `rknn-toolkit2==1.6.0`, not the current 2.3.2

The board's real on-device runtime (`/usr/lib/librknnmrt.so`, confirmed via SSH) reports
`version: 1.6.0`. A model exported with the current toolkit (2.3.2) embeds a newer model-format
version this runtime can't parse. There's no newer official RV1106 runtime to deploy instead —
checked `airockchip/rknpu2`'s public repo, and its `master` branch runtime (1.5.2) is actually
*older* than what's already on this board (Luckfox's own vendor SDK build is ahead of the generic
public tree) — so the fix is exporting with a matching toolkit version, not upgrading the device.
