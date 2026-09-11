# Changelog

Terse, dated, factual record of real changes to this project. For the full narrative/discovery log
(why, what was tried, what failed), see `PROJECT_MEMORY.md` — this file is the short version.

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
