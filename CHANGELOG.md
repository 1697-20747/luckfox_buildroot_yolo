# Changelog

Terse, dated, factual record of real changes to this project. For the full narrative/discovery log
(why, what was tried, what failed), see `PROJECT_MEMORY.md` — this file is the short version.

## 2026-10-11

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
