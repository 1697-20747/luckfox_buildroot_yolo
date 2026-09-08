# luckfox_buildroot_yolo

Investigating whether the Luckfox Pico Ultra (Rockchip RV1106G3, ~1 TOPS NPU) can run YOLO26 via
RKNN — a possible offload target for detection/positional processing that's been too slow on plain
Raspberry Pi 5 CPU. See `PROJECT_MEMORY.md` for the full real, dated history of what's actually
been tried and found — this README is just the current entry point.

**Status as of 2026-09-08: YOLO26 itself is blocked; a real, working alternative exists on this
exact board.** YOLO26 fails to even load on this board's NPU (an unsupported `MatMul` op in its
attention block) — likely an architectural limitation of this specific chip, corroborated by
Rockchip's own official support lists (RV1103/RV1106 officially supports only YOLOv5 among the
YOLO family; YOLOv8/v10/11 are all absent). But `yolov8n`, using Rockchip's own RKNN-optimized
export, **ran a real, complete inference pass on the actual hardware** — `rknn_init`, the
RV1106-specific zero-copy memory setup, and `rknn_run` all succeeded, all 9 real output tensors
correctly shaped. Decode/NMS and a real (non-dummy) test image are the next steps if this path is
pursued. See `PROJECT_MEMORY.md` for the full details.

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
