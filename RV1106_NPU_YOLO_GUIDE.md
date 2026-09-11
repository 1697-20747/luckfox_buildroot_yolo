# RV1106 NPU & YOLO, From First Principles

> Also served as a live doc site off the board itself: http://192.168.1.180:1313/docs/rv1106-npu-yolo-guide/ (or http://192.168.1.196:1313/ over WiFi). This file is the same content, kept here for plain git/GitHub reading.

## Why this exists

This board (a Luckfox Pico Ultra, Rockchip RV1106G3) started as a question: can a ~$15 SBC with a ~1 TOPS NPU run modern YOLO detection fast enough to be a real offload target for a Raspberry Pi 5 that's too slow on plain CPU? The short answer is "yes, but not the model you were hoping for, and not without understanding the memory model." This doc is the "why" behind that answer — the hardware, the toolchain, and the actual mechanics of getting a detector running on the NPU. Real numbers throughout, measured on this exact board, not vendor marketing figures.

If you just want to reproduce the working pipeline, see [Software Install, Step by Step](#software-install-step-by-step) and `README.md` in the repo root. This page is the "how does it actually work" layer underneath that.

## Hardware overview

The Pico Ultra is built around Rockchip's **RV1106G3**, a chip aimed squarely at smart-camera / IPC (IP camera) applications — that lineage matters, because it explains almost every design decision below.

| Component | Spec | Why it matters |
|---|---|---|
| CPU | Single-core ARMv7 Cortex-A7 | Everything CPU-side (decode, NMS, JPEG encode, network I/O) time-slices on **one core**. No real parallelism without offloading to the dedicated hardware blocks below. |
| NPU | ~1 TOPS, int8 | Runs quantized models only. This is a fraction of a Raspberry Pi 5's already-modest CPU inference throughput in raw compute, but it's *dedicated* silicon, so it doesn't compete with the CPU for cycles. |
| RAM | ~256MB total, ~150-200MB usable | Tight. Rules out anything that wants a large working set — this is not a chip you run a full desktop Linux distro's worth of background services on. |
| Video blocks | MPP (JPEG/H.264/H.265 **encode only** — confirmed real, up to 4MP@60fps standalone per the official RV1106 datasheet; **no hardware video decode block exists on this chip at all**, an earlier version of this doc incorrectly described MPP as also handling decode), RGA (2D scale/color-convert/rotate, max 8192×8192 source), ISP/CIF (camera pipeline) | Dedicated hardware for exactly the operations a camera application needs. Real offload attempts and results for this project's pipeline: see [Real hardware offload: RGA and MPP](#real-hardware-offload-rga-and-mpp). |
| Storage | eMMC, partitioned (env/idblock/uboot/boot/oem/userdata/rootfs) | The `boot` partition (`/dev/mmcblk0p4`) holds the raw kernel+dtb FIT image — a normal Linux system, reachable and reflashable entirely over SSH, no maskrom/USB recovery mode needed. |
| I/O | GPIO header (I2C2), CSI camera connector (I2C4), SDIO (WiFi/BT), UART (BT) | Three **separate** I2C buses in active use — don't assume they share a bus. |

The chip's whole design center is "sit inside a camera, do ISP + encode + a bit of on-device AI, ship the result." The NPU, the RGA, and the MPP encoder existing side-by-side is the tell — this is meant to run an object detector on a live ISP feed and squirt out an annotated H.264 stream, not to be a general-purpose ML accelerator. Trying to use it as one (as this project does) works, but you're swimming a little against the current of what the silicon was optimized for.

## The NPU and its runtime: the part everyone gets wrong first

Rockchip's NPU story has a trap in it that catches almost everyone coming from RK3588-class boards (Orange Pi 5, etc.): **RV1103/RV1106 uses a completely different runtime family from every other Rockchip NPU chip.**

- RK3588-class chips (and most of what you'll find tutorials for): `librknnrt.so`, the standard "full" RKNN runtime.
- RV1103/RV1106 (this board): `librknnmrt.so` — the **"mini runtime."** Different library, different C API behavior in one critical respect (below), different constraints.

This isn't a naming quirk. It reflects real hardware differences:

- **RK3588-class NPUs have full DRAM access.** The runtime can hand you a normal heap pointer for input/output tensors and the NPU just reads/writes it like any other DMA-capable peripheral.
- **RV1106's NPU has a small, DMA-mapped SRAM region instead.** It cannot arbitrarily read your process's heap. This is why RV1106's driver **only supports the zero-copy memory API** (`rknn_create_mem()` / `rknn_set_io_mem()`, using `RKNN_QUERY_NATIVE_INPUT_ATTR` / `RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR` to discover the NPU's *native* tensor layout) rather than the standard `rknn_inputs_set()` / `rknn_outputs_get()` buffer-copy API most RKNN sample code and documentation defaults to. Using the standard API on this chip fails immediately with `"context config invalid!"` — not a bug, just the wrong API for this hardware. Rockchip's own `rknn_model_zoo` reference code confirms it in a source comment: *"1106 npu only support NHWC in zero copy mode."*

### Toolkit and runtime versions have to match — literally

The on-device runtime is versioned (`librknnmrt version: 1.6.0`, built 2024-08-20, confirmed via SSH), and that version has to match the `rknn-toolkit2` version used to **export** the model, or the model fails to even load (`E RKNN: incompatible model version`). This is a real, load-bearing constraint, not a "use the latest version to be safe" situation:

- The current `rknn-toolkit2` (2.3.2 as of this writing) produces a newer model-format version than this board's runtime understands.
- There is **no newer official runtime to upgrade the device to** — Rockchip's own public `rknpu2` runtime repo's RV1106 build is actually *older* (1.5.2) than what's already on this board. Luckfox's vendor SDK build is ahead of the generic public tree.
- The fix is exporting with a **matching, older toolkit** (1.6.0 for this board) — pinned by checking `rknn-toolkit2`'s own changelog, which explicitly ties that version to RV1106-specific improvements.
- Toolkit 1.6.0 predates PyPI installability (added in 2.2.0), so it's pulled directly from its GitHub release tag, and it's an **x86_64-only wheel** (ARM64 host support wasn't added until 2.3.0) — meaning on Apple Silicon this step needs x86_64 Docker emulation, not a native arm64 container.

The practical takeaway: before exporting a model for *any* RV1103/RV1106 board, check `strings /usr/lib/librknnmrt.so | grep version` on the actual device and match your toolkit to it. Don't assume "newest is best" — on this chip family it actively breaks things.

## Software install, step by step

The full working recipe (see `docker/convert.Dockerfile` and `scripts/` in this repo), with the reasoning for each non-obvious step:

1. **Build an x86_64 Docker image** (`--platform linux/amd64` on Apple Silicon) with Python 3.10 and `rknn-toolkit2==1.6.0` installed from the GitHub release wheel (not PyPI — see above).
2. **Two environment bugs to expect, neither related to the actual conversion:**
   - `onnxruntime`'s native `.so` refuses to load under QEMU x86_64 emulation on Apple Silicon Docker Desktop (`"cannot enable executable stack as shared object requires"`). The standard fix (`execstack -c`) isn't packaged for current or EOL Debian. Fixed by patching the ELF `PT_GNU_STACK` header's executable flag directly in Python instead (`scripts/clear_execstack.py`).
   - The downloaded wheel's **exact original filename must be preserved** — renaming it breaks pip's metadata parsing (`"is not a valid wheel filename"`).
3. **Export the model**: PyTorch/Ultralytics `.pt` → ONNX (opset 12) → RKNN int8, targeting `rv1106`, using toolkit 1.6.0 (`scripts/export_yolo_rknn_1.6.0.py`). The simulator will happily pass at this stage even for models that later fail on real hardware — more on why below.
4. **Test on real hardware, not just the simulator.** This project's own history is the cautionary tale: a model that builds cleanly and passes the toolkit's built-in simulator can still fail to even *load* on the physical NPU. The simulator models the compute graph; it doesn't perfectly model every op-support edge case of the real driver+silicon stack. Always follow up a successful export with a real on-device `rknn_init()` call before trusting it.
5. **On-device compilation needs no cross-toolchain.** This board's Ubuntu image ships `gcc`/`g++` natively (glibc, not the Buildroot/uClibc SDK image some documentation assumes), so C programs against the RKNN API compile directly on the board over SSH.

## How YOLO actually runs on this chip

This is the part that took real, on-hardware trial and error to map out, because the honest picture is more nuanced than "RV1106 runs YOLO." It runs **some** YOLO.

### The architectural line: attention blocks are the wall

Cross-checking against Rockchip's own official "Current Support Platform" matrix, and confirmed independently by real on-device failures, there's a clean pattern:

| Model | Has a self-attention block? | Officially supported on RV1103/RV1106? |
|---|---|---|
| YOLOv5 | No | **Yes** |
| YOLOv8 | No | No (but works anyway — see below) |
| YOLOv10 | No | No |
| YOLO11 (C2PSA attention block) | **Yes** | No |
| YOLO26 (same C2PSA block, inherited from YOLO11) | **Yes** | No official example exists yet |

Only YOLOv5 is officially listed. Real on-device testing confirms exactly why for the newer, attention-based generations: YOLO26's C2PSA self-attention block (inherited from YOLO11) uses a `MatMul` op **inside the backbone itself**, not in postprocessing — and this op has no NPU support *and no CPU fallback* on this specific chip/driver stack:

```
E RKNN: unsupport cpu MatMul op, op name: MatMul:/model.10/m/m.0/attn/MatMul_1
E RKNN: rknn fallback cpu failed
RESULT: INIT_FAIL ret=-1
```

This fails at `rknn_init()` — model *load* time, before any inference is even attempted. It's not a speed problem, a quantization problem, or a config flag away from working: it's the physical NPU + driver stack genuinely lacking support for that operation, with the usual "fall back to the CPU" escape hatch also unavailable. The only real paths around it would be Rockchip/Ultralytics landing upstream operator support for this chip family (not tracked publicly, no movement as of this writing), or manually excising and retraining around the attention block — real model surgery, not a config change.

One counter-intuitive corollary: YOLO26/YOLOv10-class architectures are marketed as *faster* than YOLOv8 because they drop the NMS postprocessing step (a "one-to-one" head). That speedup is real — but only where NMS was the bottleneck (many-candidate-box scenarios on GPUs or larger NPUs). On this board, a real test against Rockchip's reference `bus.jpg` produced only 31 raw candidate boxes after the confidence threshold — sorting 31 boxes by IoU costs microseconds on any CPU. The actual measured bottleneck here is backbone convolution throughput on the NPU (below), and YOLO26's attention block is architecturally *more* NPU compute than YOLOv8's plain CSP backbone, not less. So even setting the load failure aside, there's no real reason to expect a win from switching — on this specific ~1 TOPS chip, the "NMS-free" pitch doesn't transfer.

### What actually works: YOLOv8, with two fixes

`yolov8n` — no attention block — loads and runs. Getting there took two separate, genuinely different fixes:

**Fix 1 — the zero-copy memory API** (covered above). Non-negotiable for this chip; the standard buffer API simply doesn't work here.

**Fix 2 — the DFL softmax head.** With zero-copy I/O working, `rknn_init()` succeeds, but `rknn_run()` then fails:

```
E RKNN: Meet unsupported input layout 3 for softmax
E RKNN: op: exSoftmax13, name: exSoftmax13:/model.22/dfl/Softmax, fallback cpu failed
```

This is YOLOv8's **DFL (Distribution Focal Loss)** box-regression head — a *postprocessing* op this time, not a backbone one, and it fails at `rknn_run()` (inference time) rather than `rknn_init()` (load time). That distinction matters: it's a much more tractable category of problem. Rockchip's own `rknn_model_zoo` confirms the fix: their reference `yolov8n.onnx` isn't the stock Ultralytics export — it comes from a customized fork (`airockchip/ultralytics_yolov8`) with the DFL head's output graph **restructured** specifically to avoid this op (splitting it into separate box-coordinate, class-score, and a reduce-sum+clip confidence-sum output instead of one softmax). Swapping in that pre-optimized ONNX export and re-running the same toolkit-1.6.0 conversion fixes it — confirmed by the output tensor names changing to match that restructured graph (`318` / `onnx::ReduceSum_326` / `331` per scale), and by real, correct detections on hardware afterward.

Worth being precise about what kind of fix this is: it's a **graph restructuring of the postprocessing head**, with zero effect on the trained weights. That's very different from what would be needed to unblock YOLO26 (removing/replacing a backbone attention block), which *would* touch trained weights and need retraining or fine-tuning. Not all "make an unsupported op go away" fixes are the same difficulty.

### The actual math: decode and NMS, by hand

The RKNN runtime gives you back raw int8 tensors — 9 of them for `yolov8n` (3 detection scales × [box coords, class scores, confidence sum]), shapes like `[1,80,80,64]` / `[1,80,80,80]` / `[1,80,80,1]` for the largest scale, with 40×40 and 20×20 equivalents for the other two. Turning that into actual boxes needs code, not a library call:

1. **Per-scale extraction**: for each of the 3 output scales, walk the grid, apply the confidence threshold, and decode the DFL box-regression distribution (`compute_dfl`) into `(x1, y1, x2, y2)` coordinates in that scale's coordinate space.
2. **Greedy per-class NMS**: sort surviving candidates by score, suppress overlapping lower-scoring boxes of the same class by IoU threshold.
3. **Rescale to display resolution**: if inference ran on a downscaled copy of the frame (see below), boxes decoded in that small copy's coordinate space get rescaled (`scale_x` / `scale_y`) before drawing on the full-resolution frame.

This project's implementation (`scripts/luckfox_yolov8_detect.c`, and the persistent-process video version `scripts/luckfox_video_hud_stream.c`) adapts this decode math directly from Rockchip's own `rknn_model_zoo` `postprocess.cc` (`process_i8_rv1106`) — real, vendor-provided reference math, not reverse-engineered from scratch. Real test against Rockchip's own reference `bus.jpg`:

```
Raw candidate boxes before NMS: 31
RESULT: 4 detection(s) after NMS
  person: score=0.847
  person: score=0.708
  person: score=0.681
  truck:  score=0.391
RESULT: DETECTION_PIPELINE_COMPLETE
```

Three people correctly localized, the vehicle correctly boxed (classified `truck` instead of `bus` — a plausible minor int8-quantization mix-up between two visually similar COCO classes; the box geometry itself, which is what this decode math is actually responsible for, is correct).

### The real performance ceiling, measured not assumed

Running this as a live, persistent video pipeline (`luckfox_video_hud_stream.c`: load the model once, set up zero-copy I/O once, loop over decoded frames — re-doing setup per frame the way the single-shot detector does would be far too slow for video) surfaces the chip's real limits:

| Stage | Cost (640×640 stream) | Cost (1280×720 native stream) |
|---|---|---|
| NPU inference alone | ~67ms/frame | ~85-93ms/frame (includes an added in-process downscale-to-640 step) |
| `ffmpeg` decode + scale | ~15-20ms/frame | ~26-34ms/frame |
| JPEG encode + network send (combined) | ~90-100ms/frame | ~150-160ms/frame |
| **Real steady-state end-to-end** | **~4.5-4.8 fps** | **~3.0-3.1 fps** |

A few honest conclusions from these numbers, not assumptions:

- **NPU inference alone caps the ceiling at ~14.7fps**, full stop — 30fps would need <33ms/frame, and inference alone is already 2× over that budget before anything else runs.
- **JPEG encode + network send is the single biggest cost in the pipeline — bigger than inference itself.** This combined figure was an open question across several sessions — see [Real hardware offload: RGA and MPP](#real-hardware-offload-rga-and-mpp) below for the real split (spoiler: it's almost entirely the encode step, not the network write, correcting an earlier guess in this doc) — and for why this is no longer true as of the libjpeg-turbo swap (NPU inference is now the biggest single cost).
- **Everything competes for one CPU core.** `ffmpeg`'s software decode, the decode/NMS math, drawing, and JPEG encoding all time-slice the same single Cortex-A7 core.
- Restricting detected classes (`ALLOWED_CLASSES = {person, bicycle, car, motorcycle, bus, truck}`, COCO ids 0/1/2/3/5/7) helps the CPU-side decode loop, but that loop was never the bottleneck, so it didn't move the needle much.

**~4.5-4.8fps is the real number for this pipeline on this hardware — not 30fps**, and that's an honest hardware/architecture ceiling, not a bug to keep chasing.

### Does the class list matter for speed? No — measured, not assumed

Easy to assume restricting the class list (fewer objects to detect) would speed up inference. It doesn't, and this is worth being precise about rather than just asserting it: `OBJ_CLASS_NUM` is hardcoded to `80` in the decode code, and `ALLOWED_CLASSES` is a **post-decode filter** — it runs after the model has already scored all 80 COCO classes for every candidate box:

```c
#define OBJ_CLASS_NUM 80
...
if (!ALLOWED_CLASSES.empty() && ALLOWED_CLASSES.find(c) == ALLOWED_CLASSES.end()) continue;
```

The real, measured 67ms/frame NPU inference cost is almost entirely backbone convolution compute — fixed by input resolution and network depth — not the classification head. The head's output channel count (80 vs. 6 vs. 1) is a tiny fraction of total FLOPs for this architecture. Full 80-class COCO already runs at the same speed a 1-class model would. If you want faster, the class list isn't the lever — input resolution is (next section).

### Input resolution: the real lever, now measured

512×512 vs. the 640×640 default, same `yolov8n` architecture, same toolkit-1.6.0/RV1106 pipeline, same test video (`bikes.mp4`), same 640×640 **display** resolution in both runs so only the inference-resolution variable changes (JPEG-encode/network-send cost tracks display pixels, not inference pixels, so holding display constant isolates the effect cleanly):

| | 640×640 inference (baseline) | 512×512 inference |
|---|---|---|
| NPU inference | ~67ms/frame | **~50ms/frame** |
| Steady-state end-to-end fps | ~4.5-4.8 fps | **~5.2-5.3 fps** |

Two honest takeaways from this, not the naive ones:

- **Inference time itself drops a real ~25%** — roughly in line with the ~36% fewer pixels (512²/640² ≈ 0.64), sub-linear because of fixed per-invocation NPU overhead that doesn't shrink with input size.
- **End-to-end fps only improves ~10-15%, well under the inference-time improvement**, because JPEG-encode+network-send (~90-100ms, unchanged here since display resolution didn't change) is still the single biggest cost in the pipeline and swamps the inference saving. Shrinking inference resolution helps some; the real dominant-cost lever is JPEG encode itself (see below).

### Real hardware offload: RGA and MPP

RV1106 has two dedicated hardware blocks that looked like the obvious next lever for this pipeline's real bottlenecks: the **2D Graphics Engine** ("RGA", `rga2` per `dmesg`, hw_version 3.3.87975, max 8192×8192 source — confirmed real against the official datasheet) for scale/color-convert, and the **MPP** video codec block for JPEG hardware encode (confirmed encode-only — no hardware decode exists on this chip at all, correcting this doc's earlier "MPP handles decode too" claim). Real discovery and two real offload attempts, one per block:

**First, the real split this doc's "JPEG encode + network send" line had left unresolved for several sessions.** Instrumenting `luckfox_video_hud_stream.c` to time `stbi_write_jpg_to_func` (encode) and `send()` (network write) separately, both over loopback and over a real WiFi client from a separate machine, gives a clean, decisive answer:

| | Combined (old) | JPEG encode alone | Network send alone |
|---|---|---|---|
| 1280×720 stream | ~150-165ms | **~146-165ms** | **~0.4-1.5ms** |

**JPEG encode (`stb_image_write`, pure software) is essentially the entire cost — network send is negligible**, even over real WiFi to a separate machine. This corrects an earlier guess in this doc ("dropping JPEG quality barely moved the number, suggesting the dominant cost is the network write") — that guess was wrong; `send()` just hands bytes to the kernel's TCP buffer and returns almost immediately, with the real WiFi transmission happening asynchronously off the hot path. This makes MPP's hardware JPEG encoder the single highest-value real target in this pipeline — bigger than NPU inference itself.

**RGA (scale/color-convert): real userspace library found, offload implemented and working, but a genuine, measured net loss on this exact hardware.** A glibc-built `librga.so.2` (the modern im2d API, `rga_api version 1.10.1`) is already present in `/usr/lib` on this board's Ubuntu image and in `ldconfig`'s cache — no build needed. Swapping `downscale_for_inference()`'s CPU nearest-neighbor loop for `imresize()` hit a real, concrete wall on the first attempt: this chip's RGA has **no IOMMU** (`rga_mm: Current NONE_MMU[0] cannot support physically discontinuous virtual address!`, a real kernel error, not a guess), so it needs genuinely physically-contiguous DMA memory, not plain `malloc()`. Fixed with a real DRM "dumb buffer" allocator (raw ioctls against `/dev/dri/card0`, no libdrm link needed — this board's `/usr/include/drm/{drm,drm_mode}.h` kernel-uapi headers are enough) — see `scripts/luckfox_video_hud_stream.c`'s `drm_alloc_contig()`. That memory turned out to be CPU-uncached, though, which is fine for RGA's own DMA access but genuinely bad for the CPU-heavy work (JPEG encode, HUD drawing, quantization) this pipeline still needs to do on the same frame data — a first working version that made the whole frame buffer DRM-backed was **dramatically slower overall** (fps 3.3→1.0) even though the RGA resize itself was fast, because every other stage touching that buffer got much slower reading through uncached memory. A second version, bouncing through two small DRM buffers only around the RGA call (copy in, resize, copy out, keeping the main frame buffers ordinary cached `malloc()`), fixed the regression elsewhere but left the resize step itself **slower than the original CPU loop it replaced**:

| | CPU nearest-neighbor (default) | RGA (`USE_RGA_DOWNSCALE=1`) |
|---|---|---|
| Downscale step alone | **~8.7-9.2ms** | ~51.6-60.5ms |
| Real steady-state end-to-end fps | **~3.3-3.4 fps** | ~2.9-3.0 fps |

The mandatory copy-through-contiguous-memory tax this no-IOMMU chip imposes outweighs what the hardware resize saves, for an operation (640×640 nearest-neighbor) that was already cheap on the CPU to begin with. This is a real, measured result, not a "didn't try hard enough" one — three different implementations were tried, each fixing the specific problem the last one's measurement revealed. The CPU path remains the default in `luckfox_video_hud_stream.c`; the RGA path is kept available (`USE_RGA_DOWNSCALE=1`) since it's real, correct, working code, just not currently the faster choice on this exact hardware.

**MPP (hardware JPEG encode): userspace library built successfully, two real integration gaps found and fixed, then a genuine kernel-level wall hit and NOT pushed through.** Unlike RGA, no glibc-compatible MPP userspace library existed on this board going in — `librockchip_mpp.so` only exists under `/oem/usr/lib` (Luckfox's vendor Buildroot/uClibc image tree), linked against `libc.so.0` (uClibc), and this Ubuntu image has no `ld-uClibc.so.0` interpreter to even run it (same real incompatibility this project's `PROJECT_MEMORY.md` already documented for the vendor's `luckfox_i2c_test` tools). The kernel side was real and active, though — `mpp_vcodec`/`rga3` both in `lsmod`, `/dev/mpp_service` present — a genuine userspace-only gap, not a deeper hardware/driver one.

Built the public `rockchip-linux/mpp` source (`develop` branch, the only current branch — the old `1.0.x`/`1.1.x` tags predate RV1106) natively on-device (`cmake`/`gcc`/`g++` all present, no cross-toolchain needed) — a real, clean, ~100%-successful build producing a working glibc `librockchip_mpp.so`. Two real, concrete integration gaps found and fixed after that, both root-caused against real hardware data rather than guessed:

1. **SoC not recognized at all.** First run: `mpp_platform: can not found match soc name` + `mpp_rt: can NOT found any allocator`. The public repo's SoC capability table (`osal/mpp_soc.c`) has no RV1106 entry — confirmed by grep, zero matches. A first fix modeled a new entry on the neighboring `rv1109`/`rv1126` chips (same IPC-camera product family) — **wrong**, caught by real hardware disagreeing once past the name-match step (below).
2. **Real hardware ID says otherwise.** `cat /proc/mpp_service/supports-device` on this board reports exactly one real device: `DEVICE[16]:RKVENC HW_ID:0x50603313` — and `0x50603313` is literally `HWID_VEPU540C` in this same source tree (`mpp/hal/inc/hal_hw_id.h`), a different, newer encoder IP block than rv1109/rv1126's vepu2/vepu541. No decode device is listed at all, consistent with this chip's datasheet (no hardware decode on RV1106). Rewrote the SoC table entry around `HAVE_RKVENC` + the `vepu540c_no_hevc` HAL (AVC+MJPEG, no HEVC — matching real RV1106 capability) instead. This got past SoC/capability matching, but JPEG encode still failed to even initialize: `mpp_enc_hal_init could not found coding type 8`. Root cause, found by reading `mpp_enc_hal_api_register()`: each individual HAL API implementation carries its OWN separate per-chip whitelist (`soc_type[]`), independent of the SoC capability table — `mpp/hal/rkenc/jpege/hal_jpege_vepu540c.c`'s `hal_jpege_vepu540c` API only lists `ROCKCHIP_SOC_RK3528`/`ROCKCHIP_SOC_RK3562`. Added `ROCKCHIP_SOC_RV1106` to that list too (justified by the same real `HWID_VEPU540C` match, not a blind guess) — got past HAL registration.

### A real, working win: libjpeg-turbo instead of stb_image_write

With hardware JPEG encode blocked by the kernel crash above, 1697-20747 asked what could ease the JPEG
encode burden without it -- the real, low-risk lever was the software encoder itself. `stb_image_write`'s
JPEG encoder is a simple single-pass implementation with no SIMD; this board's already-installed
`libjpeg-turbo8` (confirmed via `dpkg -l` -- it's the real backing implementation of this Ubuntu
image's `libjpeg.so.8`, not a separate thing to build) has hand-written ARM NEON paths for color
conversion and DCT. Installed `libjpeg-turbo8-dev` via apt for `jpeglib.h`, swapped
`http_send_frame()`'s encode call from `stbi_write_jpg_to_func` to the standard `jpeg_mem_dest()` API
at the same quality setting (50) for a fair comparison. No kernel, no register-level guessing --
purely a userspace library swap.

**Real, measured result -- this is the actual win this session was looking for:**

| | stb_image_write (software) | libjpeg-turbo (software, NEON) |
|---|---|---|
| JPEG encode alone | ~146-165ms/frame | **~50-65ms/frame** |
| Real steady-state end-to-end fps | ~3.3-3.4 fps | **~5.2 fps** |

A real ~2.5-3x reduction in encode time, and a real ~53% end-to-end fps improvement -- confirmed
correct by extracting and visually inspecting a real frame from the live stream (HUD overlay, boxes,
labels all render correctly, no corruption). **This also changes which stage is now the pipeline's
single biggest cost: NPU inference (~87-97ms) now costs more than JPEG encode (~50-65ms) for the
first time this whole investigation** -- a real, concrete answer to "what's the bottleneck now,"
not assumed. This is the current default in `luckfox_video_hud_stream.c`.

### The real kernel-level wall on hardware JPEG encode (MPP)

**Where it genuinely stopped: a kernel-level crash on the first real submitted job.** With both integration gaps fixed, a real JPEG-encode attempt (`mpi_enc_test`, synthetic 640×640 NV12 frame) got all the way to actually submitting hardware registers — and the kernel's own `mpp_vcodec.ko` module crashed inside `rkvenc_run` (a real oops, full register dump and stack trace in `dmesg`, `Process queue_work0`), followed by `rk_vcodec: mpp_wait_result_default: timeout`. The board survived (SSH stayed up, no reboot needed, the existing detection pipeline was re-verified unaffected afterward) but this is a genuine, serious wall, not a config problem: the RK3528/RK3562 register-level HAL implementation, despite matching RV1106's encoder on IP-block ID alone, does not actually submit register-compatible jobs against RV1106's real kernel driver. Matching on `HWID_VEPU540C` got the software far enough to reach real hardware, but was not sufficient for full register-level compatibility — a genuinely deeper vendor-integration detail (exact register offsets/sequencing) not discoverable from public sources. **Stopped here, deliberately, rather than continuing to probe a kernel module that had just oopsed** — real risk of an unrecoverable hang, and Luckfox's own working uClibc build (proof this chip's real hardware JPEG encode does work correctly, just not through this specific userspace/kernel combination) isn't source-available to compare register sequences against.

### Follow-up (2026-09-11, new session): testing the version-match hypothesis for real

The kernel driver's own embedded version (`cat /proc/mpp_service/version`) reports commit `424abb9b`, dated 2024-04-29 — ~2.5 years older than the `develop`-HEAD commit the userspace library above was built from. Same author on both. Real, testable hypothesis, and this project already proved this exact failure mode once for RKNN (toolkit had to be version-matched, not newest-is-best) — worth testing before assuming the wall above is final.

**The lead checked out, more precisely than expected, but not in a directly buildable way.** `424abb9b` is a real, exact commit in `rockchip-linux/mpp`'s full history — hash, message, and author all match, and its CommitDate (`2024-04-29 16:31:33 +08:00`) matches the kernel driver's reported date down to the minute. But that commit is **not reachable from `develop`** — no common ancestor at all. Its directory layout is the kernel driver's own source (`vcodec/`, `vproc/`, `mpp/mpp_vcodec_driver.c`), a completely different commit graph within the same public repo from `develop`'s userspace-library layout (`osal/`, `mpp/hal/`, producing `librockchip_mpp.so`). In hindsight this makes sense — the kernel module's version string obviously comes from building *itself*, not the separately-versioned userspace library.

Fell back to the documented alternative: the `develop`-line commit closest by date to that exact CommitDate. That's `23126cf6`, committed 2024-04-29 10:13:34 +08:00 — same day, ~6.3 hours earlier. **A date-based best-effort match, not an exact hash match** — worth weighing the result below accordingly.

Reapplied the SoC-table fixes (enum + `mpp_soc_infos[]` entry, `HAVE_RKVENC` only, `vepu540c_no_hevc`) the same way as before. The third fix (the per-HAL `soc_type[]` whitelist) **doesn't exist as a mechanism this far back** — `hal_jpege_api_v2.c` at this commit selects the JPEG HAL purely by real hardware-ID match (`HWID_VEPU540C == mpp_get_client_hw_id(...)`), no separate chip whitelist yet; that gate was added later, upstream of this commit.

Built clean, 100%, on-device (~11 min, single core). Re-ran the identical `mpi_enc_test` against the same synthetic NV12 frame, with dmesg cleared first for an unambiguous read.

**Real, different, better result: no kernel crash.** Entire dmesg output was one line — `rk_vcodec: mpp_wait_result_default:1044: timeout` — no Oops, no register dump, no stack trace. Confirmed twice (once with clean file-redirected output after the first run's stdout got lost in a pipe). Board stayed fully healthy both times.

**Still not a working encode, though.** The output file (623 bytes) has a real SOI/DQT/SOF0/DHT/SOS structure — the software-written header — but no EOI marker and no real scan data, consistent with the hardware job never signaling completion. Re-verified the existing detection pipeline is genuinely unaffected (a real run with the correct `yolov8n-optimized-rv1106-1.6.0.rknn` model: live detections, fps plateauing at ~5.0-5.1, matching the documented libjpeg-turbo baseline exactly).

**Honest conclusion: the version-match hypothesis is partially confirmed, not fully.** Moving to an era-matched userspace build turned a hard kernel panic into a soft driver-side timeout — a real, if narrow, win, and consistent with the RKNN precedent. But the encode still doesn't complete, so this remains the same underlying wall as before, just reached via a cleaner, more precisely diagnosed path: even hardware-ID-matched and now version-era-matched, the RK3528/RK3562 HAL still doesn't fully match what RV1106's real kernel driver expects at the register/sequencing/clock-power-domain level — genuinely proprietary detail with no public source to check against. One caveat on the result itself: the build used a date-matched commit, not the literal one the kernel driver was built from (unreachable from the userspace-library history entirely), so this doesn't fully rule out version skew as *a* factor — it just shows the practically-buildable nearest match isn't sufficient alone. `libjpeg-turbo` remains the current, unchanged default; `luckfox_video_hud_stream.c` was not modified this session, since there's still no working MPP encode path to integrate.

## Beyond vision: what else shares this board

Two more pieces of real, working hardware bring-up worth knowing about if you're extending this project, because they hit the same "read the real source, don't trust the first assumption" pattern as the NPU work above:

- **Accelerometer (MPU-6050, GPIO header, I2C2)**: the vendor image's kernel shipped with zero I2C adapter drivers bound to *any* bus — the framework was present, the board-specific glue wasn't. Fixed with a real device-tree patch (`&i2c2 { status = "okay"; ... mpu6050@68 {...}; }`) plus a one-line kernel Kconfig change and a small driver patch to skip IRQ-based triggered-buffer mode (this board has no INT line wired), falling back to polled reads. Real measured sample rate ended up bottlenecked by the video loop's own ~3.5fps, not the sensor or I2C bus.
- **WiFi (AIC8800DC combo chip, SDIO)**: a similar "the pieces are all there but not wired together" story — the vendor driver's Rockchip power-on path was a silent no-op without a `wlan-platdata` device-tree node providing the chip's real reset GPIO to the kernel's `rfkill-wlan` subsystem. Once wired up correctly, WiFi came up cleanly and now survives reboots via a small systemd unit. A follow-up real bug surfaced after that: the driver's SDIO sleep/wake power-management handshake (`aicwf_sdio_sleep_allow`, gated by `CONFIG_SDIO_PWRCTRL`) never actually completes on this chip/firmware combination — it busy-polls 100× at 500µs per attempt, every time, forever, burning a dedicated kernel thread's worth of CPU (confirmed via `ps`: a persistent `aicwf_pwrctl` thread, plus dmesg flooded with `warning: read cnt N` down to 0). WiFi data-path was unaffected — this is pure power-save negotiation, not connectivity — but it's real wasted CPU on a single-core board. Fixed by disabling `CONFIG_SDIO_PWRCTRL` for this driver entirely (rebuilt just the driver module, no kernel/DTB rebuild needed) rather than trying to fix an undocumented firmware handshake. Confirmed clean afterward: zero `sleep_allow` messages, no `aicwf_pwrctl` thread, WiFi still connects and holds its static IP fine.

Both are further evidence of the same underlying lesson this whole project keeps re-teaching: **this board's peripheral support is "the silicon can do it, the specific image/kernel you're holding may not have wired it up yet."** Checking `dmesg`, the real device tree, and the real running kernel config before assuming a hardware limitation saved real time, repeatedly.

## Glossary, for anyone new to this stack

- **NPU** — Neural Processing Unit, dedicated matrix-multiply silicon for running (quantized) neural networks, separate from the CPU.
- **RKNN** — Rockchip's model format + toolchain for their NPU family. `rknn-toolkit2` converts models (usually via ONNX) into a `.rknn` file; a matching on-device runtime library executes it.
- **int8 quantization** — representing model weights/activations as 8-bit integers instead of 32-bit floats. Smaller, faster on int8-native hardware like this NPU, at a small, usually-acceptable accuracy cost. All RKNN models on this chip are quantized this way.
- **Zero-copy (DMA) memory API** — instead of copying tensor data into a buffer the runtime manages, you allocate DMA-mapped memory the NPU can read/write directly. Required, not optional, on RV1106.
- **DFL (Distribution Focal Loss) head** — the part of modern YOLO models that predicts box edges as a probability distribution over discrete offsets rather than a single regressed number, decoded via a softmax + weighted sum (`compute_dfl`).
- **NMS (Non-Maximum Suppression)** — the postprocessing step that removes duplicate/overlapping detections of the same object, keeping the highest-confidence box.
- **C2PSA** — the self-attention block introduced in YOLO11's backbone (and inherited by YOLO26), using `MatMul` ops that this chip's NPU can't execute and can't fall back to CPU for.
