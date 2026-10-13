# Changelog

Terse, dated, factual record of real changes to this project. For the full narrative/discovery log
(why, what was tried, what failed), see `PROJECT_MEMORY.md` — this file is the short version.

## 2026-10-13

- **6-position accelerometer calibration (web UI: Tilt row -> Calibrate).**
  - The unit rests on each of its six faces in turn and **Capture** records it. The board picks the
    face from the dominant axis and its sign. A capture is refused while the unit moves (jitter of
    the raw readings) or isn't flat on a face (dominant axis under 90% of the total).
  - **Save** computes per axis: offset = (up + down) / 2, scale = 2 g / (up - down).
  - Applied to every raw reading before the EMA, so HUD, navigation output and status all get
    corrected values.
  - Saved to `/userdata/accel-cal.conf` (tmp + fsync + rename) and loaded at start-up, before the
    level reference. Saving a new calibration clears the stored level, which was measured with the
    old one.
  - `/api/accelcal?capture=1|save=1|reset=1|clear=1`; `cal_set` in `/api/status`.
  - Tested on the board: status (1.28 g, still), capture (+Z), save refused before six faces,
    reset. The full six-face run needs someone to turn the unit.

- **Accelerometer "Set level" button (web UI, Tilt row).** The MPU-6050 gives pitch/roll relative to
  gravity, with no drift, but in the sensor's own frame.
  - With the unit standing level, the button stores the gravity direction. Later readings are
    rotated so that direction is straight down, which stays correct for large mounting angles
    (unlike subtracting angle offsets).
  - Saved to `/userdata/accel-level.conf` (tmp + fsync + rename) and loaded at start-up. Never set
    automatically at boot: a board switched on while tilted would call that tilt level.
  - `/api/level?set=1` / `?clear=1`; `level_set` in `/api/status`. HUD, navigation output and
    status all use the corrected values.
  - Tested set (pitch/roll -> 0.0/0.0) and clear. Left cleared: calibrate once the sensor is fixed
    in place.
- **Accelerometer range log (sensor loose):** responds over the full range, but reads about
  12.5 m/s^2 at rest (about 1.27 g) and up to 1.44 g per axis. This module has offset/scale error,
  so angles are a few degrees off at large tilts. A 6-position calibration would fix it. Values are
  in m/s^2 (IIO scale), despite the `smoothed_*` "g" naming.

- **Detection 16.0 Hz while a viewer watches (was 14.9), video still 30 fps.** The NPU worker starts
  the staged frame itself the moment a job ends, instead of waiting for the main thread to notice.
  There are three NPU input tensors: one running, one staged, and one the main loop writes the
  newest frame into. Navigation events arrive at 16.1 Hz.
- **Board clock set from the browser.** The board has no RTC or network time, so it booted in 2021.
  The web UI now sends `Date.now()` to `/api/time` on load, and the board sets its clock if it is
  more than 2 s off. `yolo-healthlog` saves the time to `/userdata/.last-time` every 5 min (only
  once the clock is plausible) and restores it at boot.
- **Supervisor heartbeat age is now measured on the monotonic clock** (time since the heartbeat
  file's mtime last changed). It used to be wall clock minus mtime, so a forward clock jump looked
  like a years-old heartbeat and triggered a watchdog reset. A manual clock test did exactly that
  once (self-recovered in about 45 s). The server's NPU-hang timer is monotonic too.

## 2026-10-12

- **Video at the camera's full 30 fps; detection 14.9 Hz alongside it (17.1 Hz navigation-only).**
  - **RGA colour conversion (default with yolo_camd):** camd NV12 -> RGB 960x540 (1.1 ms vs 22 ms
    CPU), NV12 -> 800x448 model input in the same RGA pass (replaces the CPU downscale), RGB -> NV12
    straight into the hardware JPEG encoder's buffer (1 ms vs ~5). Quantise is NEON (x ^ 0x80).
    Per frame: read 4.5 ms (was 16), quant 2 ms (was 5.7), JPEG 3 ms (was 12); CPU 33% (was 86%) at
    16.7 fps. `scripts/rga_bench.c` measures it. `LUCKFOX_RGA=0`: CPU path.
  - **Every RGA buffer is a DMA-BUF imported by fd** (CMA heap / camd buffers / VENC MB block). A
    first benchmark gave the RGA plain virtual addresses: this RGA core has no IOMMU, the job was
    rejected, and the driver's error path corrupted the process's mm accounting, so the process's
    exit oopsed the kernel (panic + self-reboot; logs in `build/rga-crash-20261012`). Any RGA failure
    now switches to the CPU path for good.
  - **Decoupled display (default while a viewer watches):** each loop shows the newest camera frame
    with the latest finished detection's boxes. Every frame is also resized and quantised into the
    NPU's spare input tensor, which is submitted the moment the NPU finishes (checked every 2 ms
    while waiting for the camera). Video 30 fps (was 16.7), detection 14.9 Hz, CPU ~73% average; boxes lag the
    picture by one detection. Navigation-only clients keep the pipelined loop (17.1 Hz).
    `LUCKFOX_DECOUPLED=0` turns it off. `/api/status` adds `det_hz`.
  - 30/30 restarts clean on the RGA build and again on the decoupled build, with no RGA kernel
    errors.
  - 3 h continuous run (viewer + navigation client): 0 resets, video 30.0 fps and detection
    14.9 Hz in every streaming sample, CPU ~73% average, memory flat at 130–132 MB, 60 °C, no RGA
    kernel errors, one camera stream start.

- **Model input 800x448 (16:9) is the default, with hardware JPEG.** `tools/export_yolov8_rknn.sh 448 800`
  is a reproducible export: Rockchip's YOLOv8 fork to ONNX opset 12, letterboxed calibration images,
  then rknn-toolkit2 1.6.0. The server reads the input size from the model (`LUCKFOX_MODEL` in
  app.env; missing file or safe mode falls back to 640x640).
  - **Before:** the 640x640 model got the 960x540 frame *stretched*: rows duplicated, objects 1.78x
    too tall.
  - **Speed:** NPU 58 ms (vs 63.5); video 16.2 fps (vs 15.2); navigation only 16.7 Hz (vs 15).
  - **Accuracy:** both int8 models in the simulator on 40 bikes.mp4 frames, against float YOLOv8n at
    960 (`tools/compare_models.py`): recall 84% vs 61%, precision 92% vs 94%.
  - **Restart test:** 30/30 clean on the new build.
- **Two CPU regressions found and fixed on the way:**
  - The downscale did a hardware divide per pixel once the size was no longer a compile-time
    constant: 27 ms. A column lookup table fixed it: 6 ms.
  - The decode checked all 80 classes per cell through `std::set` lookups (588k per frame) once
    the score-sum pre-filter stopped rejecting cells; that sum's no-object floor (0.27) is above
    the 0.25 threshold on this model. Checking only the allowed classes from an array fixed it:
    20.5 ms down to 1.8 ms. A NEON 80-class quick reject covers an empty class list.
  - Diagnostics kept: per-output layout/quant log at start-up, `LUCKFOX_DUMP_OUTPUTS=1` (one-shot
    raw output dump), `post_cpu` and `copy` timings in the per-frame log.
- **Hardware JPEG is the default** (`LUCKFOX_PIPELINE=hwjpeg`, `LUCKFOX_HW_JPEG=1`) after a 3 h
  continuous run: 0 resets, 13.8–14.9 fps, memory flat.
- **`/api/status` timings are 30-frame rolling averages** (were single-frame values).

## 2026-10-11

- **3 h continuous run (yolo_camd, software JPEG):** viewer + navigation subscriber attached
  throughout, 0 resets, 13.1–14.1 fps, free memory flat at 126–128 MB, one camera stream start
  (`tools/continuous_soak.sh`).
- **Hardware JPEG re-tested on the yolo_camd base: passes.** Opt-in binary
  `luckfox_video_hud_stream_hwjpeg` (`LUCKFOX_PIPELINE=hwjpeg` + `LUCKFOX_HW_JPEG=1` in app.env).
  - 50 of 50 server restarts clean, each one tearing down Rockit (VENC + `RK_MPI_SYS_Exit`) while
    the camera stream kept running. The earlier "hardware JPEG freezes" were most likely the
    camera-restart eMMC hang.
  - JPEG 11.7 ms (vs 29), CPU 75% (vs 100%), 14.7 fps (vs 13.9). NPU-bound now.
  - Software JPEG stays the repo default until a long continuous run with hardware JPEG.
- **docs/ARCHITECTURE.md:** short install and architecture summary.

- **Camera stream no longer restarts with the server (`yolo_camd`).** Every eMMC hang followed a
  camera stream restart, and the first stream start of a boot never failed. So `scripts/yolo_camd.c`
  (static, about 250 lines) now opens `/dev/video11` once at boot and never stops it. It exports the 4
  capture buffers as DMA-BUFs to the stream server over a unix socket (`\0yolo-camd`, SCM_RIGHTS). Per
  frame the server borrows the newest buffer, converts it to RGB and gives it back, with no copy.
  yolo-app-start starts 3A then camd once; a server restart touches neither. Only if camd dies are 3A
  and the stream restarted (boot order).
  - Result: 100 of 100 server restarts clean, with one `stream ON` for the whole run. Before:
    option A about 1 hang in 33 restarts, persistent 3A 8 in about 55.
  - Video unchanged: 13.95 fps.
  - `LUCKFOX_CAMD=0` in app.env goes back to direct capture.

- **Kernel experiment (not adopted): ISP writes wait for eMMC DMA idle.**
  `buildroot/kernel-patches-experimental/0004` makes the ISP/VICAP register writes that Rockchip
  guards with `rv1106_sdmmc_get_lock()` disable IRQs and wait, up to 10 ms, for the eMMC IDMAC to go
  idle. That lock is a no-op on this single-core PREEMPT_NONE kernel.
  - Persistent-3A restart test: 1 hang in 40, same signature, with no guarded write logged before
    the stall. So those writes are not, or not alone, the trigger.
  - Board back on kernel #7; the kernel #7 boot image is kept at `/userdata/boot-k7.img`.

- **Correction: option A is a mitigation, not a fix.** The 100-cycle soak hung at cycle 3 (same
  signature), so A is about 1 failure in 33 restarts, against 1 in 5 with a persistent 3A server.
  The eMMC controller state at the hang shows a write (CMD25) with 0 of 32 blocks moved, TXDR pending
  and the FIFO empty while the card is idle: the controller's internal DMA stopped feeding it. No clock
  rate changes across a restart. `yolo_supervisor` now maps the controller (/dev/mem 0xffa90000) at
  start-up and dumps all its registers, including the IDMAC ones, on a storage fault.
  `LUCKFOX_3A_RESTART=0` in app.env keeps one 3A server across restarts (experiments only).
  Vendor report draft: docs/vendor-report-emmc-hang-on-camera-restart.md.

- **Option C tested and rejected; board back on option A.** Rockit VI capture with in-process 3A
  started cleanly (correct exposure, 14.1fps, same as A) but hung in restart cycle 2 with the same
  signature: eMMC re-initialised 5.5s after the camera stream restart, write I/O error, journal
  aborted. So the trigger is the 3A library's activity on a stream restart in whichever process runs
  it; what option A changes is the timing -- a fresh 3A process gets ~2s to settle before the stream
  starts. Summary: no 3A 0/30, option A 0/30, persistent 3A 5/~25, option C 1/2. Reverted via
  app.env (LUCKFOX_PIPELINE removed). Also: S05watchdog now actually installed with its debugfs
  mount (an earlier tar transfer had failed), so storage faults capture the eMMC controller registers.

- **Board hangs: root cause localised by evidence; option A mitigation (see correction above).** New tooling first:
  `yolo_supervisor` (static, memory-locked) owns the hardware watchdog and resets the board when the
  stream server's heartbeat goes stale or a /userdata write+fsync stalls -- after dumping every task's
  kernel stack (SysRq, `kernel-patches/0003`: SysRq + 128KB log buffer) and the eMMC controller
  registers into the RAM crash log, which survives the warm reset. Hangs now self-recover in ~45s.
  Crash logs from real hangs showed the same sequence every time: **~5.8s after the camera stream is
  restarted, the eMMC stops completing writes** (`mmc0` re-initialised, `I/O error ... WRITE`,
  journal aborted, every task blocked in `io_schedule`). Ruled out by test or data: NPU activity,
  hardware JPEG, the server's own work, eMMC runtime PM (A/B), IR-cut switching, 3A writes to storage,
  worn flash (EXT_CSD life 0-10%, pre-EOL normal). Restart stress results: long-running
  rkaiq_3A_server across stream restarts **5 failures in ~25**; no 3A server **0/30**; fresh 3A
  server started before every stream start (the boot-time order, which never failed in 12+ boots)
  **0/30**. `yolo-app-start` now does the latter. The vendor 3A binary is closed source, so the
  mechanism inside it isn't visible; the fix reproduces the configuration that never failed.
- **Option C prepared, off by default:** `luckfox_video_hud_stream_rockit` -- Rockit VI capture with
  in-process 3A (rk_aiq uAPI2), mirroring Rockchip's sample_vi. `LUCKFOX_PIPELINE=rockit` in
  app.env selects it; default, unset, or safe mode = option A.

- **Root filesystem survives power loss:** after repeated hard power cycles the board came up with
  a READ-ONLY root ("Couldn't remount RDWR because of unprocessed orphan inode list": stock Luckfox
  mounts root ro, then inittab remounts rw, which ext4 refuses while orphans are pending). Repaired
  with e2fsck on the ro root + `reboot -f`; fixed for good by booting with `rw` on the kernel command
  line (`apply_overlay.sh` patches `project/build.sh`; env partition rewritten live, old one backed
  up), so ext4 replays its journal on mount. Verified: cmdline has `rw`, root rw at boot.
- **Stream server feeds the hardware watchdog itself** (arms after start-up, kicks with the
  heartbeat unless an NPU job is overdue, disarms after full teardown); NPU worker thread blocks
  SIGTERM/SIGINT/SIGPIPE/SIGALRM. Restart stress: 18/19 clean (was ~1 in 3-5 failing); the 19th
  lost the USB link without a watchdog reset, i.e. the server kept running -- `yolo-healthlog`
  now records usb0 counters/state, heartbeat age and USB kernel messages to /userdata every 5s.

- **Three-stage NPU/CPU pipeline:** an NPU worker thread runs YOLO on frame N while the main
  thread captures/prepares N+1 and draws/encodes N-1 with N-1's own detections (3 frame buffers,
  2 NPU input tensors). Measured: video **7.8 -> 14.2fps**, navigation-only **10.3 -> 15.0Hz**
  (NPU 61 -> 64ms; CPU now fully used). 3-minute soak with viewer + subscriber: flat 14.1fps,
  2,610 frames / 2,612 events, max event gap 99ms, 60C.
- **Memory leak fixed (old, also on Ubuntu):** the JPEG output buffer was passed back with the last
  frame's compressed length as its size, so libjpeg reallocated whenever a frame grew and never
  freed the old buffer -- ~14KB/frame, +200KB/s; the process had grown to 52MB. Now one buffer of
  known capacity: flat 11.3MB over 90s of streaming.
- **Board freeze NOT specific to hardware JPEG:** it froze during a software-mode stop/restart cycle
  (5th of 10) with Rockit not even loaded. Every freeze so far involves stopping/restarting the
  camera stream while the separate rkaiq_3A_server process is running. Under investigation.

- **WiFi off by default, with an on/off button** in the web UI (Board card, `/api/wifi`).
  `/usr/bin/yolo-wifi on|off|status` stops/starts rkwifi_server, wpa_supplicant and wlan0 DHCP;
  `yolo-app-start` runs `yolo-wifi boot-off` once Luckfox's background WiFi bring-up settles.
  Verified across a reboot (off by 30s, still off at 72s); USB stays up throughout. Credentials:
  `/userdata/wpa_supplicant.conf` still holds Luckfox's placeholder network.
- **Navigation output:** `/api/detections` and `/api/detections/stream` (SSE) -- per frame: track id,
  class, score, normalised bounding-box centre (approximate centre of mass) and box, pitch/roll,
  optional bearing/elevation from `LUCKFOX_CAM_HFOV_DEG`/`VFOV_DEG`. A subscriber runs the detector
  without a video viewer and skips HUD + JPEG: 10.3Hz detections vs 7.7fps with video.

- **JPEG: libjpeg-turbo (NEON), statically linked; hardware JPEG dropped.** 7.7fps, JPEG 27ms
  (IJG libjpeg 9: 5.4fps / 83-93ms). `tools/build_jpeg_turbo.sh` cross-builds 2.1.4, sha256-pinned
  to the SDK's own Buildroot package. The app no longer links or loads Rockit at all.
- **Hardware JPEG freeze investigation, concluded.** Debug kernel (`kernel-patches/0002`): hardware
  watchdog fed by `S05watchdog`, ramoops, soft-lockup panic; self-test passed (feeder killed -> reset
  in 24s, log survived). `tools/hwjpeg_stress.sh` + kernel-log step markers from the app. Findings:
  freezes during Rockit init, streaming and `RK_MPI_SYS_Exit()` teardown (last kernel line
  `vi_release`), even with the V4L2 stream stopped first. The kernel is only half-dead -- USB
  networking gone but the watchdog feeder alive -- so the board never resets and pstore stays empty;
  only a power cycle recovers. Most likely cause: Rockit's media stack used alongside raw V4L2
  capture on the same ISP, which Rockchip's own software never does. Side findings: the
  hung-task panic I first enabled panicked every boot (~60s) because rockit's `vlog`/`valloc`/`vsys`
  threads idle in D state -- removed from the patch and disabled at boot.
- **`apply_kernel_patches.sh` fixed:** run inside the kernel subdirectory, `git apply` silently
  ignored the patches' paths, so applying and the "already applied" check did nothing. Now applies
  from the SDK root with `--directory`; verified on a pristine tree. Patch 0002 regenerated clean.
- **Deploy no longer overwrites files in use:** `deploy_app.sh` unpacks to a staging dir and
  renames into place (the four freezes before the stress tooling all followed a deploy that
  overwrote the running binary/libjpeg in place).

- **`docs-site/` removed from the repo** (kept locally, now gitignored): the Hugo mini site is
  board-only by choice. Earlier commits still contain it.

- **Hardware JPEG switched back OFF by default** (`scripts/app.env`: `LUCKFOX_HW_JPEG=0`; software
  libjpeg, 5.4fps). Stopping the server in hardware mode proved unsafe after all: once
  `RK_MPI_SYS_Exit()` hung on a futex (process stuck holding port 8080; now bounded by a 3s
  `alarm()`), once the whole board froze during a deploy -- 2 bad stops in ~9. That freeze also left
  the just-deployed binary and libjpeg as zero-byte files (never flushed), so `deploy_app.sh` now
  `sync`s before stopping the old server. Software-mode restarts verified clean mid-stream.
- **Main page "Docs" link**, shown only if a docs site is installed at `www/docs/` on the board. The
  site itself (Hugo, built on the Mac) is deliberately kept out of this repo: it lives in the
  gitignored `local/` folder with its own deploy script.

- **HUD tilt instruments now live from the MPU-6050** (were grey "no orientation source" stubs):
  pitch/roll from the smoothed gravity vector (`atan2`), feeding the horizon ladder (shifts with
  pitch, rotates with roll), the roll-ring pointer and the P/R readouts; also `pitch`/`roll` in
  `/api/status` and a "Tilt" row in the web UI. Compass tape stays a stub (heading needs a
  magnetometer or gyro integration). Readout boxes now size to their text (bitmap glyphs ~1 font
  size wide). No zero-offset for the mounting yet: angles are relative to the sensor's own axes.

- **MPU-6050 accelerometer ported to the Buildroot kernel.** The Ubuntu-era kernel fix had only
  ever lived in a scratch SDK clone; it's now `buildroot/kernel-patches/0001-mpu6050-i2c2-polled-mode.patch`
  (applied by `buildroot/apply_kernel_patches.sh`): i2c2 enabled with an `mpu6050@68` node (header
  pins 32/33, no INT line), `CONFIG_INV_MPU6050_I2C=y`, and the driver allowed to run without an IRQ
  (direct/polled mode). On this kernel the missing IRQ fails earlier than on Ubuntu (at
  `irq_get_irq_data`, before the trigger setup), so both points are guarded. Flashed live over SSH
  (`dd` of `boot.img` to `/dev/mmcblk0p4`, sha256-verified, previous boot partition backed up to the
  Mac first). Verified: `2-0068` on i2c-2, `iio:device1` = `mpu6050`, live readings, stream server
  `ACCEL_FOUND`, HUD accelerometer panel live; 8.7fps (hardware JPEG), CPU 40%.

## 2026-10-10

- **Board moved from Ubuntu 22.04 to Luckfox's Buildroot SDK image** (Pico Ultra eMMC config,
  SDK `824b817`), built on the Mac in Docker (`docker/sdk.Dockerfile`, amd64, SDK in a
  case-sensitive volume), flashed from the Mac in maskrom with `tools/flash_buildroot.sh`. Reason:
  on Ubuntu the camera had no auto-exposure (vendor `rkaiq_3A_server` is uClibc-only), raw MPP
  oopsed the kernel, RAM was tight. Ubuntu is recoverable byte-for-byte: verified full eMMC image
  in `board_backup/` + `tools/restore_emmc.sh` (write path verified from macOS).
  Login is now `root@172.32.0.70` by SSH key only (all passwords locked).
- **Build fixes:** GNU make 4.4.1 in the SDK image (make 4.3's jobserver died with "write
  jobserver: Bad file descriptor" under emulation, three times); `buildroot/fix_ownership.py`
  hooked into the SDK's `mkfs_ext4.sh` (building as uid 1000 had made every rootfs file uid 1000,
  `/etc/shadow` 0755, setuid lost).
- **Rootfs overlay `buildroot/overlay-yolo`:** USB NCM at `172.32.0.70` (macOS needs it), SSH key,
  Rockchip's demo `rkipc` replaced by `/usr/bin/yolo-app-start` (3A server + our stream server,
  restart loop, boot-loop guard), telnetd/Samba disabled, ADB's always-on TCP 5555 firewalled.
- **Camera exposure fixed:** vendor 3A server with the sensor's IQ file; mean luma 24 -> 127.
- **Stream server cross-built for Buildroot** (`tools/build_app.sh`, `tools/deploy_app.sh`).
  Repo's `scripts/apex_hud_widgets.h` was stale vs the board's copy; restored.
- **Hardware JPEG via Rockit VENC:** 8.9fps at 960x540 (JPEG 13ms, CPU 44%, 135MB free) vs 8.5fps
  on Ubuntu with libjpeg-turbo. Two board freezes found and fixed on the way: RGB888 VENC input
  (now off unless `LUCKFOX_VENC_FMT=rgb`) and `killall` mid-encode (now a graceful SIGTERM
  shutdown, 5/5 stop-while-encoding rounds clean). Full-range NV12 conversion (limited range looked
  washed out).
- **Not ported yet:** MPU-6050 accelerometer (device-tree fix), docs site. WiFi (AIC8800) works out
  of the box on Buildroot.

- **USB from a Mac is now a permanent connection option** (`ssh pico@172.32.0.70`,
  `http://172.32.0.70:8080/`). Root cause it didn't work: stock `/etc/init.d/S50usbdevice` exposes
  the network over RNDIS, which macOS has no driver for. Patched it to CDC-NCM (kernel already has
  `CONFIG_USB_CONFIGFS_NCM=y`), created first so it keeps the `usb0` name, with fixed dev/host MACs
  (`02:4c:46:00:00:70` / `a2:77:da:49:66:9e`) so the Mac keeps its static `172.32.0.1/16` config.
  ADB function unchanged. Copy in `board/etc/init.d/S50usbdevice`; stock original at
  `/root/S50usbdevice.rndis.orig` on the board. Verified: real reboot (Mac reconnected with no
  manual step), `S50usbdevice restart` (the path `rc.local` runs on USB disconnect), and the stream
  serving HTTP 200 over USB with `luckfox-video-stream` active.

## 2026-09-20

- **Connectivity decision: single viewer, point-to-point, no dedicated router.** Dropped the
  paused "multi-client fan-out" idea (noted 2026-09-11) instead of leaving it open. Documented
  (`README.md`, source comments in `luckfox_video_hud_stream.c`/`_cam.c`) that the existing
  `INADDR_ANY`/`listen(fd, 1)` server already serves either WiFi or the board's existing `usb0`
  USB-gadget network interface (one cable for power + data) unchanged — no code change required.
  A mini travel router stays explicitly out of scope unless one of those two paths proves unusable
  on-device. Re-confirming `usb0` while the stream service is actually running is the next
  on-device check, not done this session.

## 2026-09-11

- **Board: U-Boot env `sys_bootargs` changed** — `rk_dma_heap_cma` 36M → 66M (matches the board
  config's own `RK_BOOTARGS_CMA_SIZE` default, which had never actually been applied to this board's
  flashed env). Backed up first (`/dev/mmcblk0p1`, 32KB, md5-verified in two places). Verified: real
  reboot, same kernel build, all hardware healthy, `CmaTotal` 47MB→78MB as predicted. Does **not**
  fix RGA+live-camera (see below) — that failure is against a different, DTB-hardcoded 10MB
  `linux,cma` region, not this one. Left in place as a real, harmless improvement.
- **Fixed for real: RGA downscale + live camera's `DRM_IOCTL_MODE_CREATE_DUMB: Cannot allocate
  memory`.** Root cause: DRM's dumb-buffer allocator draws from a separate, hardcoded-in-DTB 10MB
  `linux,cma` reserved-memory region (`rv1106-luckfox-pico-ultra-ipc.dtsi`, `size = <0xA00000>`),
  shared with the live ISP/CIF pipeline's own buffers — not the `rk_dma_heap_cma` heap above. Resized
  to 32MB (`0x2000000`), rebuilt (`./build.sh kernel`, same proven Docker SDK checkout as the
  accelerometer/WiFi kernel work), backed up (boot partition + env, md5-verified), reflashed, and
  verified: real ~9s reboot, `CmaTotal` came back at exactly 66M+32M=98MB, all hardware healthy, zero
  regression on the default CPU-downscale path. First confirmed the currently-running `boot.img` has
  no actual FIT signature `value` (declared but empty) before touching anything — direct proof this
  board's U-Boot doesn't enforce signature verification, so an unsigned repack was safe.
- **Result once RGA could actually run: still a net loss, now proven under a second, harder
  condition.** `downscale_ms` ~35-36ms (RGA) vs. ~9.3-9.5ms (CPU) on the same live-camera setup —
  RGA is ~4x slower, not a CPU-contention artifact. `USE_RGA_DOWNSCALE` stays off by default.
- **`scripts/luckfox_video_hud_stream.c`: live camera capture wired in.** Auto-detects `/dev/videoN`
  vs. a video file; real V4L2 mmap capture loop (NV12→RGB24) replaces `ffmpeg` file-decode for that
  mode, opened once and kept alive across HTTP reconnects. Verified end-to-end on real hardware
  (sc3336 sensor, 1920x1080) — HUD, detections, and live accelerometer readout all render correctly
  in a pulled frame.
- **Fixed: latent `SIGPIPE` crash in the HTTP server**, present since the file's original design
  (masked previously by `ffmpeg`'s own "Broken pipe" message on client disconnect). One-line fix
  (`signal(SIGPIPE, SIG_IGN)`), verified on both the file and live-camera paths.
- **MPP hardware JPEG encode: version-match hypothesis tested, partial result.** Built userspace MPP
  from the `develop`-line commit closest by date to the kernel driver's own embedded build commit.
  Result: the kernel-level Oops from the prior session is gone (a soft driver timeout instead) — real
  progress — but the encode still doesn't complete. `libjpeg-turbo` remains the default; no code
  change.
- **Re-verified:** the 512x512-vs-640x640 inference benchmark against today's `libjpeg-turbo` build,
  and confirmed via live `top` sampling that the CPU was fully saturated (0% idle) on the old
  `ffmpeg`-based file pipeline — no longer the case once the live-camera path removed `ffmpeg`.
