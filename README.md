# luckfox_buildroot_yolo

Investigating whether the Luckfox Pico Ultra (Rockchip RV1106G3, ~1 TOPS NPU) can run YOLO26 via
RKNN — a possible offload target for detection/positional processing that's been too slow on plain
Raspberry Pi 5 CPU. See `PROJECT_MEMORY.md` for the full real, dated history of what's actually
been tried and found — this README is just the current entry point.

> **2026-10-10: the board now runs Luckfox's Buildroot image, not Ubuntu.** Much of this README
> describes the Ubuntu era (the `pico` user, systemd units, apt, on-device `g++`). Current setup:
> `ssh root@172.32.0.70` (key only) over the USB cable; web UI at `http://172.32.0.70:8080/`,
> started at boot by `/usr/bin/yolo-app-start`; build on the Mac with `tools/build_app.sh`, deploy
> with `tools/deploy_app.sh`; rebuild/flash the OS with the SDK in Docker (`docker/sdk.Dockerfile`,
> `buildroot/`, `tools/flash_buildroot.sh`). Back to Ubuntu: `tools/restore_emmc.sh` with the image
> in `board_backup/`. Live camera now has real auto-exposure, and JPEG is hardware-encoded: 8.9fps.
> See `CHANGELOG.md` / `PROJECT_MEMORY.md` 2026-10-10.

**Why this whole pipeline is plain C, not Python: a deliberate choice, not a lack of Python.**
`python3` (3.10.12) is installed on the board, and precompiled armhf Debian packages exist for both
`python3-opencv` (`apt-cache policy` confirms `4.5.4+dfsg-9ubuntu4` on `ports.ubuntu.com`, no
from-source build needed) and `python3-numpy` — so Python + OpenCV genuinely *could* run here.
The reason this project doesn't use them: the board has only **147MB total RAM** and a single real
CPU core (confirmed live, `free -m`/`nproc`), and the current bare-C pipeline already runs close to
that ceiling on its own. A Python interpreter plus OpenCV's full native library chain (a dozen-plus
`libopencv-*.so` dependencies) would add real, measured-nowhere-but-obviously-nontrivial memory and
per-frame interpreter overhead on top of that, for a board whose entire design center is "small
embedded footprint" — not a hypothetical concern here, a hardware constraint actually hit multiple
times this project (see the CMA/memory-sizing entries in `PROJECT_MEMORY.md`).

**Want the "how does this actually work" explainer instead of the session-by-session log?** See
[`RV1106_NPU_YOLO_GUIDE.md`](RV1106_NPU_YOLO_GUIDE.md) — hardware, the RKNN toolchain, and why some
YOLO generations run on this chip and others don't. Also served live, straight off the board, at
`http://<board-ip>:1313/docs/rv1106-npu-yolo-guide/` (a small Hugo site running on-device — see
[`docs-site/`](docs-site/)).

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

**Re-checked 2026-09-11, after the live camera + V4L2 pipeline was fully working: still blocked,
same exact failure, confirming this is unrelated to the pipeline itself.** `yolo26n` still fails at
`rknn_init()` — before any camera/HUD/streaming code even runs — with the identical error as the
original investigation: `unsupport cpu MatMul op, op name: MatMul:/model.10/m/m.0/attn/MatMul_1`,
`rknn fallback cpu failed`. **Is this solvable at all?** Not a hard law-of-physics wall, but not
worth pursuing on this board: (1) waiting for Rockchip/Ultralytics to add the op — the tracked
upstream issue has had zero activity in 8+ months; (2) graph-surgery YOLO26's C2PSA attention block
into NPU-compatible ops yourself (the same kind of surgery Rockchip already did for YOLOv8's DFL
head) — genuinely possible, but the pretrained weights stop applying once the computation changes,
so it needs real fine-tuning/retraining afterward, a standalone ML project, not a patch; (3)
different, more capable Rockchip silicon (RK3566/RK3588-class already has official YOLO11 support).
**Even solving it likely wouldn't help:** YOLO26/YOLOv10+'s speed advantage is mostly about dropping
NMS postprocessing overhead, but on this exact chip NMS was never the bottleneck (~31 candidate boxes
survive threshold on the real test image — microseconds of work); the real cost is backbone
convolution (NPU MAC throughput), and C2PSA's attention block adds *more* of that, not less.
`yolov8n` remains the right choice for this board, not a placeholder waiting to be replaced.

## Live HUD video stream over WiFi

`scripts/luckfox_video_hud_stream.c` runs a persistent detection pipeline against a video file and
serves it as a live MJPEG stream with an always-visible HUD overlay (boxes, labels, fps/inference-
time readout — apex_hud-inspired, not a literal port). Display resolution is decoupled from
inference resolution: the NPU always sees a 640x640 downscaled copy (inference cost stays
constant), while the stream/HUD renders at the source video's own native resolution by default —
real, measured tradeoff: full native res looks better but costs real fps (JPEG encode scales with
pixel count — see below, network send itself turns out to be negligible). See `PROJECT_MEMORY.md`
for the full profiled breakdown at both resolutions; 30fps is not achievable at either.

**JPEG encode vs. network send, split for real (2026-09-11):** the combined "encode+send" figure
above had been an open question for several sessions — instrumenting the two separately (and
testing over a real WiFi client, not just loopback) shows JPEG encode (`stb_image_write`, software)
is essentially the entire cost, ~146-165ms/frame at 1280x720; network send is ~0.4-1.5ms, negligible
even over real WiFi. This makes RV1106's hardware JPEG encoder (part of its MPP video block) the
highest-value real offload target in this pipeline — see `RV1106_NPU_YOLO_GUIDE.md`'s "Real hardware
offload: RGA and MPP" section for the full real investigation. **RGA downscale offload:** implemented,
correct, but a measured net loss on this hardware (off by default, `USE_RGA_DOWNSCALE=1` opts in).
**MPP hardware JPEG encode:** real userspace-only library gap found, the public `rockchip-linux/mpp`
source was built natively on-device and two real integration gaps (missing SoC table entry, missing
per-HAL soc whitelist) fixed using real hardware-ID cross-referencing — but the first real submitted
job crashed the kernel's `mpp_vcodec.ko` module (oops in `rkvenc_run`, board survived, no reboot
needed). Stopped there deliberately rather than keep probing a module that had just oopsed.
**Follow-up (2026-09-11):** tested the obvious next hypothesis — the kernel driver's embedded version
(`424abb9b`, 2024-04-29) is ~2.5 years older than the `develop`-HEAD userspace build above, the same
version-mismatch failure mode this project already solved once for RKNN. Real result, partial
confirmation: rebuilding userspace MPP from the nearest-date `develop`-line commit (`424abb9b` itself
turned out to live in a disjoint, kernel-driver-only history with no common ancestor with `develop` —
a real, more precise finding than expected) genuinely changed the failure from a hard kernel panic to
a soft driver-side timeout, no crash at all. But the encode still doesn't complete — the output JPEG
has valid headers and no actual image data. Real forward progress, still not a working encoder. See
`RV1106_NPU_YOLO_GUIDE.md`'s MPP section for the full account. `libjpeg-turbo` remains the default.

**Real, working win instead: libjpeg-turbo replacing stb_image_write, same quality setting.** No
kernel involved — this board's `libjpeg-turbo8` (the real backing implementation of its installed
`libjpeg.so.8`) has ARM NEON paths `stb_image_write`'s simple, unoptimized encoder doesn't. Real,
measured result: JPEG encode dropped from ~146-165ms to **~50-65ms/frame**, and real steady-state fps
rose from ~3.3-3.4 to **~5.2fps**. This also flips which stage is now the pipeline's single biggest
cost — NPU inference (~87-97ms) is now bigger than JPEG encode, for the first time in this
investigation. This is the current default.

```
sudo ./luckfox_video_hud_stream <model.rknn> <video.mp4> [labels.txt] [port] [display_w] [display_h]
```
`display_w`/`display_h` default to the source video's own resolution (auto-probed via `ffprobe`) —
pass them explicitly (e.g. `960 540`) to trade quality for fps. Then open
`http://<board-ip>:<port>/` in a browser on the same WiFi network (or over USB — see below).

**Connectivity, decided 2026-09-20: single viewer, point-to-point, no dedicated router required.**
This project only ever needs one laptop looking at one stream — there's no plan to fan a single
stream out to multiple simultaneous viewers (the "multi-client fan-out" idea noted as a paused,
separate target in `PROJECT_MEMORY.md`'s 2026-09-11 RGA/MPP entry is dropped, not just deferred),
and the server already reflects that: `listen(server_fd, 1)` and a single `accept()` in both
`luckfox_video_hud_stream.c` and `luckfox_video_hud_stream_cam.c` were already single-client by
design, not a limitation to fix. Because both bind `INADDR_ANY` rather than a specific interface,
the same binary already serves this over whichever link is up, with no code change needed:
- **WiFi (current default path):** `http://<wifi-ip>:<port>/`, e.g. `http://192.168.1.196:8080/` —
  works today, as documented above.
- **USB (preferred going forward — one cable for both power and data, no access point needed):**
  `ssh pico@172.32.0.70` / `http://172.32.0.70:8080/` over the same USB cable used to power the
  board. **Verified working from a Mac 2026-10-10, across a real reboot and with the stream service
  running.** The stock gadget used RNDIS, which macOS has no driver for (the Mac saw an `rk3xxx`
  USB device but never got a network interface), so `board/etc/init.d/S50usbdevice` switches it to
  CDC-NCM (native on macOS, Linux and Windows 11) with fixed MAC addresses. One-time Mac setup:
  `networksetup -setmanual "rk3xxx" 172.32.0.1 255.255.0.0` — macOS remembers it, and the fixed
  host-side MAC keeps it the same interface every boot.
- **Mini travel router:** intentionally not used unless one of the two paths above turns out to be
  unusable on-device for a reason not yet known (e.g. `usb0` not surviving reboot, or a laptop's
  USB-gadget driver not cooperating) — not a planned part of this project's normal path.

**Real, measured on-device (2026-09-13): dropping display/capture resolution is the biggest
remaining fps lever, and it costs nothing in detection quality.** Inference always runs on a
downscaled 640x640 copy regardless of display size (`downscale_for_inference()`), so display
resolution and detection resolution are already independent — lowering the former doesn't touch the
latter at all. Live-camera (`luckfox_video_hud_stream_cam`, `/dev/video11`) steady-state, same
`yolov8n-optimized` model, same board:

| Display res | fps | read_ms | downscale_ms | infer_ms | jpeg_ms |
|---|---|---|---|---|---|
| 1920x1080 | 4.2-4.3 | ~46 | ~9.3 | ~75 | ~106-108 |
| **960x540 (current default)** | **8.4** | ~11.8 | ~8.3 | ~73-74 | ~27 |
| 640x480 | 9.7 | ~7 | ~8.0 | ~73-74 | ~16 |

`infer_ms` is unchanged across all three (73-75ms, within noise) since the NPU always sees the same
640x640 input regardless of display size — the fps difference comes entirely from JPEG-encode/read
cost scaling with display pixel count. **`960x540` is the shipped default as of 2026-09-13**
(`luckfox_video_hud_stream_cam.c`'s live-camera fallback when no `display_w`/`display_h` args are
given) — chosen as the fps/HUD-quality middle ground between the 4.3fps 1080p baseline and the
9.7fps 640x480 extreme; pass explicit `display_w`/`display_h` args to override. The ISP's stepwise
sensor sizes accept all three resolutions directly via `VIDIOC_S_FMT` with no renegotiation.

**Runs automatically from boot as of 2026-09-13** via `/etc/systemd/system/luckfox-video-stream.service`
(same convention as `aic8800-wifi.service`/`luckfox-docs.service`) — starts
`luckfox_video_hud_stream_cam` against `/dev/video11` on port 8080 with no resolution args (so it
uses the 960x540 default above), `Restart=on-failure`. Verified across a real reboot, not just
`systemctl start`. `sudo systemctl status luckfox-video-stream.service` / `journalctl -u
luckfox-video-stream.service -f` to check it; `sudo systemctl stop luckfox-video-stream.service`
before manually running a different build/resolution for testing, or the service will hold
`/dev/video11`/`/dev/rknpu` and the manual run will fail.

**HUD overlay ported from `apex_ros`'s `apex_hud` package (2026-09-13), in plain C** —
`apex_hud_widgets.h`/`.c` reimplements 4 of its 14 widgets (the ones with a real Luckfox data
source; the rest need concepts this board has no data for at all — orientation/attitude, lidar
range, a path/session/lap context — and are intentionally not stubbed in, since a fabricated
placeholder for a concept that doesn't exist here would be worse than omitting it):
- `apex_hud_draw_crosshair` — static center reticle, direct port of `widgets/crosshair.py`
- `apex_hud_draw_track_boxes` — real YOLO detections (bbox/class/confidence); no azimuth/distance
  (needs `apex_core.camera_lens` + #32's mono-distance work, not done on this board yet)
- `apex_hud_draw_accelerometer` — real port of `widgets/accelerometer.py`'s polar-grid dot, fed by
  this board's own MPU-6050 reads (replaces the old plain-text accel readout)
- `apex_hud_draw_radar_map_stub` — `widgets/radar_map.py`'s grid/box always draws, honestly grey
  with zero objects plotted (no real per-object x/y/z distance exists on this board yet)

Same color language as `apex_hud/style.py` (green instrument color, cyan track boxes, grey
no-data), same "always draws, grey when no data" contract for the fixed-slot widgets. Real,
measured zero fps cost — 8.3fps before and after, the added draw calls are noise against the
73-75ms NPU inference floor. See `PROJECT_MEMORY.md`'s 2026-09-13 "apex_hud port" entries for the
full real test, including a screenshot-equivalent frame check.

**Full-HUD follow-up pass, same day**: added the remaining orientation/heading widgets 1697-20747 asked
for by name, all in their real apex_hud "no data" neutral/grey state (this board has no orientation
source at all — MPU-6050 raw accel only, no fusion/magnetometer):
- `apex_hud_draw_horizon_stub` — `ArtificialHorizonWidget` port: pitch ladder + fixed chevron
- `apex_hud_draw_roll_ring_stub` — `RollRingWidget` port: tick arc + pointer at roll=0
- `apex_hud_draw_compass_tape_stub` — `CompassTapeWidget` port: heading tape at heading=0, "---" readout
- `apex_hud_draw_attitude_readouts_stub` — `AttitudeReadoutWidget` port: boxed "P--"/"R--"

Plus two things not from `apex_hud` itself: a real `cpu %` line (system-wide, `/proc/stat` delta,
same technique `top` uses) added to the top-left diagnostic panel, and `apex_hud_font_size()` —
every text size in this HUD now scales with `display_h` against a 960x540 reference, instead of a
fixed pixel size tuned for only one resolution. `RolloverMarkerWidget` remains deliberately not
ported — it has no meaningful neutral default (per its own real contract) and this board has no
rollover-angle source either. Real, verified: `-O2` rebuild (127236 bytes, a sane +4888 over the prior 122348-byte build — not
the earlier missing-optimization mistake), fps unchanged at 8.3, and a real frame pulled off the
live stream confirmed the ladder/ring/tape/readout boxes all render. One known minor legibility
issue: the roll ring's major-tick labels ("60" etc.) render a bit garbled at their current tiny
font size on this custom bitmap font — bump `apex_hud_draw_roll_ring_stub`'s label base size
(currently 8) up a couple points if that's worth fixing.

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

**Physically wired as of 2026-09-13** — pin 40 (`GPIO1_A1`) now has a real jumper to the MPU-6050's
`INT` pin for interrupt-driven data-ready signaling. **Not yet functionally active**: confirmed on
the board same-day that `/sys/bus/iio/devices/iio:device1` (the `mpu6050`) still has no trigger
files, and the kernel driver patch in this repo's `PROJECT_MEMORY.md` still unconditionally skips
IRQ-based triggered-buffer setup and falls back to plain polled sysfs reads (`in_accel_*_raw`) —
which is all `luckfox_video_hud_stream.c` actually uses. Wiring the line alone doesn't flip this;
the kernel/devicetree side still needs the skip-IRQ patch reverted (re-enabling
`devm_request_irq()`/triggered-buffer mode) before the hardware connection does anything. Real next
step if lower-latency/jitter sampling is wanted, not done yet.

## Live camera

**Connected and verified working as of 2026-09-11.** A real `sc3336` (SmartSens SC3336, 3MP) sensor
is on the CSI connector — this is exactly the one sensor this board's vendor image already ships a
driver for, so the earlier IMX219/IMX708/OV5647-compatibility concern below never applied here.
Confirmed for real, not from config alone: `4-0030` on I2C bus 4 is bound to the real `sc3336` kernel
driver (`i2cdetect -y 4` shows `UU`, in-use, at `0x30`), `rkcif`/`rkisp` are both loaded and already
have a client (`rockit`), and a real `v4l2-ctl` capture at 1920x1080 NV12 (installed `v4l-utils` via
apt — not on the image by default) produced an exact, correctly-sized frame that decodes to a real,
recognizable scene (checked by pulling it to a PNG and looking at it, not just trusting a clean exit
code — the raw grab was quite dark, most likely unset auto-exposure/gain on this direct `v4l2-ctl`
path rather than anything wrong with the sensor). Real max resolution: **2304x1296**, stepwise from
32x32 in 8px steps, across NV12/NV21/NV16/UYVY/RGB565/BGRX and more (`/dev/video11`-`/dev/video18`,
the `rkisp_mainpath` capture nodes).

**Wired into `luckfox_video_hud_stream.c` for real as of 2026-09-11.** Passing `/dev/video11` (the
`rkisp_mainpath` capture node) as the video argument now switches the same binary into a real V4L2
mmap capture loop at 1920x1080 NV12, converted in-process to RGB24 (same binary still accepts a video
*file* too, auto-detected by whether the path starts with `/dev/video`). Verified genuinely
end-to-end, not just a clean build: pulled a real frame out of the live MJPEG stream and looked at it
— HUD overlay, live fps/infer readout, and a real live accelerometer panel all rendering correctly.
Real, measured steady-state: **~4.3fps** at 1920x1080 live (vs. ~5.2-5.3fps at 1280x720 from a file) —
slower, but for a real, expected reason (JPEG encode scales with pixel count, and 1920x1080 is 2.25x
the pixels of 1280x720), not a regression; NPU inference actually got a real ~13% *faster* with no
`ffmpeg` subprocess left competing for this chip's one CPU core. See `PROJECT_MEMORY.md` for the full
account, including a real latent `SIGPIPE` bug in the HTTP server (present since this file's original
design, previously masked by `ffmpeg`'s own "Broken pipe" message on disconnect) found and fixed along
the way.

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
- `board/etc/init.d/S50usbdevice` — the board's USB-gadget init script, patched from stock RNDIS
  to CDC-NCM so a Mac gets a USB network link (installed at `/etc/init.d/S50usbdevice`; stock
  original backed up on-board at `/root/S50usbdevice.rndis.orig`).
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
