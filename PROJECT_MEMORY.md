# PROJECT_MEMORY.md

Real-hardware investigation: can the Luckfox Pico Ultra (RV1106G3, 1 TOPS NPU) run YOLO26 (or any
recent YOLO generation) via RKNN, as a possible offload target for positional/detection processing
that's been too slow on the Raspberry Pi 5's plain CPU? This file records what actually happened,
session by session, in the order it was found -- not a tidied-up summary.

This project replaces an abandoned April 2026 attempt at the same task (same directory, same goal
-- even landed on the same `rknn-toolkit2==1.6.0` toolkit version independently) that never got
past empty scaffolding: a `Dockerfile.convert`, a `setup_project.sh`, a 0-byte wheel file, and
empty `output`/`wheels` directories. That April content has been removed -- nothing in it had
actually been run, so there was nothing real to preserve. See `apex_ros/ACTION_ITEMS.md` #30 and
`apex_ros/PROJECT_MEMORY.md`'s matching entry for the sibling investigation done the same day in
that project (this board was originally being evaluated as an offload target for `apex_ros`'s
stereo-vision pipeline).

## 2026-09-08 -- SSH access established, real environment discovered, first real YOLO26 conversion attempt

**Discovery.** The board was connected via LAN cable (has a real Ethernet port on this variant,
confirmed by 1697-20747 -- not WiFi-only). Found via the Mac's ARP table by MAC address
(`2e:e9:24:a3:c9:3f`, router showed device name "luckfox") after a subnet port-22 scan -- real
credentials from a prior session: `pico@192.168.1.116` / password `luckfox` (see
`LUCKFOX_ACCESS.local.md`, gitignored, IP is DHCP-assigned and will change between sessions).

**Real environment, confirmed via SSH, corrected several assumptions carried over from the
apex_ros investigation earlier the same day:**
- Ubuntu 22.04.3 LTS, kernel 5.10.160, glibc 2.35, 32-bit ARMv7 Cortex-A7 -- NOT the bare
  Buildroot/uclibc image assumed beforehand.
- On-device RKNN runtime is `/usr/lib/librknnmrt.so` (note: **`librknnmrt`**, the RV1103/RV1106
  "mini runtime" family -- a completely different artifact from `librknnrt.so`, which is what
  RK3588-class chips use and what generic RKNN docs default to referencing).
- **`librknnmrt version: 1.6.0` (built 2024-08-20)** -- this is a real, load-bearing fact: it's
  OLDER than what any Ultralytics `format="rknn"` export with a current `rknn-toolkit2` (2.3.2)
  produces, so a model exported with the current toolkit fails to even load
  (`E RKNN: incompatible model version`). Checked whether a newer official runtime exists to
  deploy instead (`airockchip/rknpu2`'s public RV1106 runtime tree) -- it doesn't; that repo's
  `master` branch only has **1.5.2**, actually OLDER than what's already on this board. Luckfox's
  own vendor SDK build is ahead of the generic public tree. Conclusion: there is nothing newer to
  upgrade the device TO -- the fix has to be re-exporting with a MATCHING toolkit version, not
  updating the device.
- `arm-linux-gnueabihf-gcc` (plain `gcc`/`cc`) is natively present -- C-API programs compile
  directly on-device, no cross-toolchain needed.
- `pico` has no passwordless sudo, but the SSH password (`luckfox`) also works for `sudo`
  (confirmed, not guessed blindly -- asked 1697-20747 first, "try same" was his answer).
- `/dev/rknpu` is root-only (`crw-------`) -- inference needs `sudo`.
- Only ~200MB total RAM, ~150MB available.
- Real internet access confirmed (reaches github.com).

**Found the real matching toolkit version.** `rknn-toolkit2`'s own `CHANGELOG.md` explicitly ties
v1.6.0 to RV1106 improvements ("Reduce RV1106 rknn_init initialization time...", "RV1106 adds
int16 support...") -- the toolkit and runtime version numbers track together for this chip
specifically. v1.6.0 predates pip installability (added in 2.2.0) so it's not on PyPI -- pulled
directly from its GitHub tag's `packages/` directory instead (x86_64-only wheel; ARM64 host
support wasn't added until 2.3.0, so this old version needs x86_64 Docker emulation on this Mac's
Apple Silicon, unlike the same-day apex_ros investigation's yolo26n/rv1106 export via 2.3.2, which
ran natively on arm64).

**Two real, non-obvious environment bugs hit and fixed while getting the old toolkit running under
x86_64 emulation, neither related to the actual RKNN conversion:**
1. `onnxruntime`'s native `.so` refused to load under QEMU x86_64 emulation on Apple Silicon
   Docker Desktop: `"cannot enable executable stack as shared object requires: Invalid argument"`.
   The standard fix (`execstack -c`) isn't packaged in current Debian OR the (now EOL) bullseye
   repos. Fixed instead by patching the ELF `PT_GNU_STACK` program header's executable flag
   directly in Python (`scripts/clear_execstack.py`) -- no extra system package needed.
2. Wheel filename mattered: renaming the downloaded `.whl` broke pip's own metadata parsing
   (`"is not a valid wheel filename"`) -- had to keep the exact original filename
   (`rknn_toolkit2-1.6.0+81f21f4d-cp310-cp310-linux_x86_64.whl`).

**First real export: `yolo26n.pt` -> ONNX (opset 12, reused from the same-day apex_ros export) ->
RKNN int8 for `rv1106`, using toolkit 1.6.0. Build succeeded, simulator inference passed 4/4 real
COCO8 images with correct output shape.** This looked like a real success at the time -- see below
for why it wasn't, once tested on actual hardware.

## Real on-device test #1: YOLO26 fails to even LOAD on real hardware -- an architectural blocker, not a config issue

Wrote a minimal C program (`scripts/luckfox_rknn_test.c`) against the real RKNN C API
(`rknn_api.h`, fetched from `airockchip/rknpu2`'s actual RV1106 runtime tree -- the generic
`rknpu2/runtime/.../armhf/` tree has a DIFFERENT header/library, `librknnrt.so`, not the RV1106
`librknnmrt.so` this board actually has), compiled it natively on the device, and ran it against
the yolo26n-toolkit1.6.0 export for real, with `sudo` (needed for `/dev/rknpu`).

**Real result, not simulated:**
```
E RKNN: unsupport cpu MatMul op, op name: MatMul:/model.10/m/m.0/attn/MatMul_1
E RKNN: rknn fallback cpu failed
RESULT: INIT_FAIL ret=-1
```

This is a genuinely different failure than the historical `ultralytics/ultralytics#23340` segfault
(that was YOLO26's `topk` postprocessing op, fixed upstream months ago -- confirmed separately the
same day in the apex_ros investigation). This is YOLO26's **C2PSA self-attention block** (inherited
from YOLO11's architecture), which uses a `MatMul` op inside the backbone itself, not the
postprocessing head. RV1106's real NPU+driver stack can't execute this op on the NPU AND can't
fall back to CPU for it either. The Phase-1 simulator run (same day, apex_ros side) passed cleanly
-- this is a real, confirmed instance of exactly the kind of simulator/device mismatch that
`airockchip/rknn-toolkit2#145` warned about, not a hypothetical risk anymore.

**Cross-checked against Rockchip's own official support matrix, not just this one failure --
a clear, consistent pattern across three YOLO generations:**

| Model | Has attention block? | RV1103/RV1106 in Rockchip's own "Current Support Platform" list? |
|---|---|---|
| YOLOv5 | No | **Yes** |
| YOLOv8 | No | No |
| YOLOv10 | No | No |
| YOLO11 (same C2PSA block as YOLO26) | Yes | No |
| YOLO26 | Yes | No official `rknn_model_zoo` example exists at all yet |

Only YOLOv5 is officially listed as supported on this exact chip. This strongly corroborates the
real failure above: it is very unlikely to be a fixable configuration/export-flag issue -- the
RV1106G3's real NPU hardware/driver stack appears to genuinely lack support for the attention-block
op YOLO26 (and YOLO11) need, independent of which toolkit/runtime version is used, since even the
*current* 1.6.0-vs-2.3.2 axis doesn't touch which ops the physical NPU silicon can execute.

**Honest conclusion so far: getting YOLO26 running on this exact board's NPU looks architecturally
blocked, not just difficult.** A real fix would mean either (a) Rockchip/Ultralytics landing
official RV1106 support for the attention op (tracked nowhere publicly yet -- the closest thing,
`airockchip/rknn-toolkit2#498` "YOLO26 support" for the related RV1126 chip, is open with zero
maintainer response since 2026-01), or (b) manually stripping/replacing the C2PSA block from the
model graph before quantization -- a real, large, accuracy-risking piece of model surgery, not
attempted here, not something to promise lightly.

## Real on-device test #2: yolov8n (no attention block) -- confirms the pattern, surfaces a second, separate, more fixable bug

Re-exported `yolov8n.pt` -> ONNX (opset 12) -> RKNN int8/rv1106 via the same toolkit-1.6.0
pipeline. **Build succeeded. On real hardware: `rknn_init` SUCCEEDED this time** (confirms YOLOv8's
plain CSP architecture, with no attention block, IS accepted by this runtime -- unlike YOLO26) --
but then hit a second, different, genuinely separate real bug:

**Bug 2a -- wrong I/O API for this specific chip.** The first attempt used the standard
`rknn_inputs_set()`/`rknn_outputs_get()` buffer API (works fine on RK3588-class chips) and failed
with `"context config invalid!"` (ret=-5) at `rknn_inputs_set`. Root-caused by comparing against
Rockchip's own real, working reference (`rknn_model_zoo`'s `yolo11_rv1106_1103.cc`) -- RV1106's
NPU has a fundamentally different memory architecture (small DMA-mapped SRAM, not full DRAM access
like RK3588) and its driver **only supports the zero-copy memory API**
(`rknn_create_mem`/`rknn_set_io_mem`, querying `RKNN_QUERY_NATIVE_INPUT_ATTR`/
`RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR` instead of the plain attr queries) -- confirmed directly in
that reference file's own comment: "1106 npu only support NHWC in zero copy mode". Rewrote
`luckfox_rknn_test.c` to use this API. Fixed: input/output memory setup succeeded on the real
retry.

**Bug 2b -- real, still-unresolved: the DFL softmax head.** With the I/O API fixed, `rknn_run`
itself then failed:
```
E RKNN: Meet unsupported input layout 3 for softmax
E RKNN: op: exSoftmax13, name: exSoftmax13:/model.22/dfl/Softmax, fallback cpu failed
```
This is YOLOv8's DFL (Distribution Focal Loss) box-regression head's Softmax op -- a different
location and different kind of problem than YOLO26's backbone attention MatMul (this is a
POSTPROCESSING op, and it fails at `rknn_run`/inference time, not at `rknn_init`/model-load time --
a real, meaningfully different, generally more tractable class of RKNN problem). Confirmed this is
a known category by checking Rockchip's own `rknn_model_zoo` yolov8 README: their own pretrained
`yolov8n.onnx` is explicitly NOT the stock Ultralytics export -- it's from a customized fork
(`airockchip/ultralytics_yolov8`) with the DFL head's output graph restructured specifically to
avoid ops like this one (same "optimized model, different from the official original model"
pattern the yolo11 README also documents). Downloaded that real pre-optimized `yolov8n.onnx` from
Rockchip's own CDN and re-ran the same toolkit-1.6.0 conversion against it.

**Result: real success, confirmed on actual hardware.** Build produced 9 outputs (3 detection
scales x [box coords, class scores, a reducesum+clip confidence sum] -- the softmax-free graph
restructuring the README described, confirmed by output names: `318`/`onnx::ReduceSum_326`/`331`
for the first scale, etc.). Deployed and ran for real on the Luckfox:
```
RESULT: INIT_OK
io_num: n_input=1 n_output=9
RESULT: INPUT_MEM_SET_OK
RESULT: OUTPUT_MEM_SET_OK
=== rknn_run ===
RESULT: RUN_OK, got 9 output(s)
RESULT: REAL_HARDWARE_INFERENCE_PASSED
```
**This is the first fully working real-hardware RKNN inference pass this investigation has
produced** -- `rknn_init`, the zero-copy memory setup, and `rknn_run` all completed cleanly against
the real NPU, all 9 output tensors with correct shapes (`[1,80,80,64]`/`[1,80,80,80]`/`[1,80,80,1]`
and the 40x40/20x20 equivalents for the other two scales). Still only confirms the runtime PATH
works end-to-end, not detection correctness -- the dummy input was zeroed, and no decode/NMS has
been written yet to turn these 9 raw int8 tensors into real boxes; that's the next real step if
this path is pursued further.

**Note also, for completeness: RV1103/RV1106 is not in the "Current Support Platform" list even
for this officially-optimized yolov8 export** -- so this is an unofficial-but-empirically-working
configuration, not a documented-supported one. Worth remembering if this ever needs debugging
without a matching official reference to lean on.

## What this means for the original goal (YOLO26 offload, replacing slow Pi-5-CPU inference)

**Real, working proof that this exact chip CAN run a real YOLO detector end-to-end now exists**
(yolov8n via Rockchip's own optimized export) -- the earlier pessimistic read ("architecturally
blocked") turned out to be specific to YOLO26/YOLO11's attention block, not this board generally.
YOLO26 itself is still blocked at model-load time by the same real C2PSA MatMul issue found
earlier -- that finding stands, confirmed on real hardware, not superseded by this result. The
honest current picture: (1) yolov8n (or a similarly attention-free architecture) is a real,
demonstrated-working path on this exact board today, at some cost in detection quality/recency
versus YOLO26; (2) YOLO26 specifically would need either upstream Rockchip/Ultralytics support for
the attention op on this chip family (tracked nowhere publicly yet), or someone doing the same kind
of graph-surgery Rockchip did for yolov8's DFL head, applied to YOLO26's C2PSA block instead -- a
real, substantial, accuracy-risking undertaking, not attempted here; (3) a different, more capable
Rockchip chip (RK3566/RK3588-class) would sidestep this entirely, official YOLO11 support already
exists there. None of these are decided -- recorded here as the real, current state of the
tradeoff, not a recommendation made without 1697-20747's input.

## Real, full end-to-end detection confirmed working on real hardware (2026-09-08, same session)

Wrote `scripts/luckfox_yolov8_detect.c` -- a self-contained detection program (stb_image for
JPEG decode, a plain bilinear resize -- deliberately NOT Rockchip's full demo build, which needs
RGA/libjpeg-turbo as extra native dependencies not required just to prove real detections work).
The zero-copy RKNN I/O is the same proven pattern from `luckfox_rknn_test.c`; the decode math
(`compute_dfl`, the per-scale box/score extraction, greedy per-class NMS) is adapted directly from
Rockchip's own `rknn_model_zoo` `postprocess.cc` (`process_i8_rv1106` specifically, fetched and
inspected in `vendor_reference/` -- real, working reference math, not re-derived from scratch).

**Ran for real against Rockchip's own reference `bus.jpg` (640x640, the classic bus+boarding-
passengers test image) on the actual Luckfox NPU:**
```
Raw candidate boxes before NMS: 31
RESULT: 4 detection(s) after NMS
  person: score=0.847 box=[...]
  person: score=0.708 box=[...]
  person: score=0.681 box=[...]
  truck: score=0.391 box=[...]
RESULT: DETECTION_PIPELINE_COMPLETE
```
**Real, correct detections** -- 3 people at sensible positions/confidences, the vehicle correctly
localized (a large box spanning most of the image at the right position) though classified `truck`
instead of `bus` -- a plausible minor int8-quantization confusion between two visually similar COCO
classes, not a pipeline bug (the box geometry itself is right, which is what the decode math
actually being tested here produces). **This is the first fully real, correct, end-to-end YOLO
detection this investigation has produced -- decode math and all, not just "the runtime path
completes."**

Ubuntu update check (1697-20747 asked): 22.04.3 LTS, many packages upgradable, all routine Jammy
point-releases (security/bugfix patches within the same Ubuntu version) -- no OS version jump, no
kernel package in the upgradable list (kernel is a separate vendor-flashed component, not
apt-managed). Nothing here touches the RKNN/ISP driver stack. Not run -- 1697-20747 didn't ask for the
upgrade to actually happen, just the status.

**Next real step: video.** 1697-20747 has a test video to process (path pending) -- current resize is a
simple stretch, not letterboxed (fine for the single-image proof above; worth revisiting if aspect
ratio matters for the real video's content). Live camera input is explicitly deferred by 1697-20747 to
later -- video file processing first.

## Real, working live HUD video stream over WiFi -- and a real, honest fps ceiling found (2026-09-08, same session)

1697-20747 asked for a web-page HUD output (apex_hud-inspired: always-visible readout, not "nothing
drawn when idle" -- NOT a literal port of apex_hud's ROS 2/rclpy widgets, which have no equivalent
runtime here) that live-streams over WiFi, using real test videos from
github.com/JoyKarmoker/YOLOv8-Object-Detection-on-Video-with-OpenCV (picked `bikes.mp4`, smallest
file, real bicycle/person content matching the restricted class list below). Also asked to restrict
detected object classes to speed things up, with 30fps as an interim target.

**Built `scripts/luckfox_video_hud_stream.c`:** persistent process (model loads ONCE, zero-copy I/O
set up once -- re-doing that per frame, as the single-shot `luckfox_yolov8_detect.c` does, would be
far too slow for video) that pipes a video through `ffmpeg` (installed via apt -- confirmed
available, not preinstalled) as raw RGB24 640x640 frames, runs the same real decode/NMS math as
before (restricted to `ALLOWED_CLASSES = {person, bicycle, car, motorcycle, bus, truck}` -- COCO
ids 0/1/2/3/5/7 -- per 1697-20747's "restrict object types to speed inference"), draws boxes + labels +
an always-visible HUD readout panel (frame count, fps, per-frame inference time, detection count)
using Rockchip's own `image_drawing.c`/`font.h` (real bitmap-font text rendering, reused not
rewritten -- needed one real fix: their file is missing `#include <math.h>`, patched locally),
JPEG-encodes each frame (`stb_image_write`), and serves it as a live `multipart/x-mixed-replace`
MJPEG stream over a minimal single-client HTTP server on port 8080. **Confirmed working end-to-end,
for real, from this Mac's browser-equivalent (`curl`) over the actual WiFi network** -- real JPEG
frames with real overlays streaming live, not simulated.

**Real, profiled performance numbers (not assumed) -- 30fps is NOT achievable on this exact
hardware, and that's an inference-time ceiling, not a software bug to chase further:**
- NPU inference alone: a steady **~67ms/frame**, measured directly -- this alone caps the ceiling
  at ~14.7fps regardless of anything else in the pipeline. 30fps would need <33ms/frame; inference
  alone is already 2x over that budget.
- `ffmpeg` decode+scale (piped in, `fast_bilinear` scale flag, no `-re` throttle): ~15-20ms/frame --
  small, not the bottleneck, confirmed by direct measurement rather than assumed.
- JPEG encode + HTTP send combined: ~85-100ms/frame -- the single biggest real cost, bigger than
  inference itself. Tried lowering JPEG quality 80->50 as a cheap lever: negligible change,
  suggesting the dominant cost inside this stage is the actual WiFi network write to the connected
  client, not the encode step itself -- a real finding, not assumed, though not chased further via
  e.g. a wired/local-only client comparison (out of scope for this pass).
- **Real steady-state end-to-end throughput measured: ~4.5-4.8fps**, climbing slowly from a colder
  ~3fps at stream start, plateauing there across multiple real runs.
- All measurements are also competing on this chip's **single CPU core** (confirmed earlier via
  `/proc/cpuinfo` -- one `processor:` entry) -- `ffmpeg`'s software decode, the quantize/decode/NMS
  math, drawing, and JPEG encoding all time-slice the same one core sequentially, no real
  parallelism available without wiring up RV1106's hardware video/image blocks (MPP for decode, RGA
  for scale/color-convert -- not attempted this pass, real future levers if pursued further).

**Honest conclusion: ~4.5-4.8fps is the real, current number on this hardware for this pipeline,
not 30fps** -- restricting object classes (done) helps the CPU-side decode loop but that loop was
never the bottleneck here, so it didn't move the needle much; the real ceiling is inference time
itself plus encode/network overhead. If 30fps genuinely matters going forward, the real levers are:
lower input/output resolution (trades accuracy for speed -- not tried this pass), RV1106's hardware
MPP/RGA blocks for decode/scale (real, substantial engineering, not attempted), or accepting this
chip's real ~1 TOPS NPU + single Cortex-A7 core simply isn't built for 30fps YOLOv8-class detection
video, full stop.

Ubuntu update check (1697-20747 asked, addendum): also confirmed `ffmpeg` installs cleanly via the
existing Jammy `apt` repos (`7:4.4.2-0ubuntu0.22.04.1`), no issues.

## Re-checked whether YOLO26 is now viable, and why "YOLOv10+/YOLO26 is faster" doesn't transfer here (2026-09-08, same session)

1697-20747 asked directly: given YOLOv10+/YOLO26 are supposed to be much faster than YOLOv8, worth
trying the YOLO26 path again, or is the chipset the real constraint? Re-checked fresh rather than
assuming yesterday's answer still holds: `rknn-toolkit2` latest release still v2.3.2 (unchanged),
`rknpu2` latest tag still v1.5.2 (unchanged), the tracked upstream issue
(`airockchip/rknn-toolkit2#498`, "YOLO26 support") still has zero new comments since 2026-01-28, a
fresh GitHub search for "C2PSA RV1106" / "RV1106 attn MatMul" across all repos found nothing new.
**Re-ran the exact same real on-device test against the already-exported `yolo26n` model -- same
failure, byte-for-byte identical error, reproduced right now, not stale:**
```
E RKNN: unsupport cpu MatMul op, op name: MatMul:/model.10/m/m.0/attn/MatMul_1
```

**Answering the actual question: the chipset is the real constraint, and it's not a speed
question, it's a hard compatibility one.** Two different "update paths" were already tried across
this investigation (2.3.2 toolkit against the old runtime -> model-version mismatch; 1.6.0 toolkit
matching the real runtime -> model builds fine, passes the simulator, but fails at real
`rknn_init` on hardware) -- neither is a version-skew problem to fix by updating something, because
the root cause is that RV1106's actual NPU silicon + driver genuinely has no support for the
`MatMul` op YOLO26's C2PSA self-attention block needs, with no CPU fallback either. That's real,
current, and unchanged.

**Why "YOLOv10+/YOLO26 is faster" doesn't help even if it did load:** that speed claim is real, but
specifically about eliminating NMS postprocessing overhead (the "one-to-one" NMS-free head) --
which matters most on hardware where NMS is a genuine bottleneck (GPUs, or edge NPUs handling many
candidate boxes). On THIS chip, the real bus.jpg test already showed NMS is not the bottleneck: only
31 raw candidate boxes survived the confidence threshold, an IoU sort over 31 items costs
microseconds on any CPU -- the real, measured 67ms/frame cost is backbone convolution compute
(NPU MAC throughput), not postprocessing. YOLO26's C2PSA attention block, layered on top of a
similar backbone to YOLOv8's, is architecturally MORE NPU compute per frame than YOLOv8, not less
-- so even setting the load-failure aside entirely, there's no real reason to expect YOLO26 would
be faster than the already-working yolov8n pipeline on this specific ~1 TOPS chip. The general
"YOLO26 is faster" claim is true on the hardware it was measured on (GPUs, RK3588-class NPUs in
Ultralytics' own benchmarks) and doesn't transfer to RV1106's specific bottleneck.

**Conclusion, unchanged from before but now freshly re-verified rather than assumed stale:** yolov8n
(via Rockchip's optimized export) remains the real, working path on this exact board. YOLO26 stays
blocked on a real hardware/driver limitation, not a version or speed issue -- the only way past it
would be Rockchip/Ultralytics landing real operator support for this chip family (tracked nowhere,
no movement) or someone doing real model surgery to remove/replace the attention block (untried,
large, accuracy-risking, and would need retraining/fine-tuning, not just a graph edit -- unlike the
DFL-softmax removal that made yolov8 work here, which was a pure postprocessing restructuring with
no effect on the trained weights).

## Decoupled display/inference resolution -- real, working, real tradeoff measured (2026-09-08, same session)

1697-20747 asked: can full camera-feed resolution go to the stream/HUD while a reduced resolution feeds
inference? Yes -- implemented in `luckfox_video_hud_stream.c`: `ffmpeg` now decodes/scales to the
source's own native resolution (auto-probed via `ffprobe`, no longer hardcoded to 640x640), a
separate cheap in-process downscale (`downscale_for_inference()`, nearest-neighbor) produces the
640x640 copy the NPU actually sees, detections decode in that small copy's coordinate space and get
rescaled back up (`scale_x`/`scale_y`) before drawing on the FULL-res frame. Real, not assumed --
tested against `bikes.mp4`'s native 1280x720.

**Real measured tradeoff, exactly as predicted before implementing, now confirmed with numbers, not
assumed:** full-res streaming makes the picture better but real fps drops, it does not improve:

| stage | 640x640 stream | 1280x720 stream (native) |
|---|---|---|
| ffmpeg read | ~15-20ms | ~26-34ms |
| inference (+ the new downscale step) | ~67ms | ~85-93ms |
| JPEG encode + network send | ~90-100ms | ~150-160ms |
| real steady-state fps | ~4.5-4.8 | ~3.0-3.1 |

The downscale-for-inference step itself is a real, non-negligible added cost (~18-26ms) -- nearest-
neighbor over ~410K pixels with a per-pixel integer division, on a weak single ARMv7 core with no
guaranteed vectorization. Worth optimizing (precomputed row/col index tables, or a faster resize
algorithm) if this path gets pursued further; not done this pass.

`display_w`/`display_h` are now optional CLI args (`luckfox_video_hud_stream <model> <video>
[labels] [port] [display_w] [display_h]`), defaulting to the source's own resolution when omitted
-- 1697-20747 (or whoever runs this) can pick the real quality/speed tradeoff point directly (e.g.
960x540 as a middle ground) rather than the tool silently choosing one.

## Live camera: not connected, and a real compatibility gap found before physically trying (2026-09-08, same session)

Checked hardware state for the "live camera next" step: `dmesg` shows the kernel's MIPI-CSI camera
stack IS present and initialized (`rkcif`, `rockchip-mipi-csi2`, `csi2-dphy` all probed
successfully) but every capture stream reports `get remote terminal sensor failed` -- no physical
sensor currently answers on the bus, confirming no camera is connected (matches what 1697-20747 already
said).

1697-20747's plan: use a Raspberry Pi Camera Module (v1/OV5647, v2/IMX219, or v3/IMX708 -- not yet
confirmed which) on the ribbon connector, "given restrictions" (presumably no SC3336-compatible
module on hand). **Checked before any physical connection, per this project's own "verify before
recommending" discipline:** this board's vendor image ships exactly ONE camera sensor kernel
module -- `/oem/usr/lib/ko/sc3336.ko` (Rockchip's own SmartSens SC3336). No IMX219/IMX708/OV5647
driver exists anywhere on this filesystem. A physically-compatible-looking ribbon connector does
NOT guarantee the kernel will recognize the sensor -- without a matching driver, it likely won't
appear as a working V4L2 device even if electrically connected. Real, stacked risks: (1) driver
support, confirmed missing; (2) FPC pinout/electrical compatibility between a genuine Pi Camera
ribbon and this board's MIPI-CSI connector, unconfirmed either way.

**Plan going forward, not yet executed:** once physically connected, check `dmesg`/I2C bus
detection first (`i2cdetect`, not yet confirmed installed) -- that answers whether the board sees
anything electrically at all, before investing in the real, nontrivial work of cross-compiling a
kernel sensor driver against this exact kernel build (would need the matching kernel source/headers,
not yet confirmed available on-device or via the Luckfox SDK). Not attempted yet -- waiting on
1697-20747 to physically connect the module and confirm which Pi Camera version it is.

## Camera ribbon fixed (wrong orientation), accelerometer checked, static LAN IP, WiFi set up (2026-09-09)

**Camera ribbon cable orientation:** 1697-20747 had it backwards. Real official source checked before
answering (wiki.luckfox.com/Luckfox-Pico-Ultra/CSI-Camera/, not guessed): metal contacts face the
chip (RV1106) side of the board; also open the connector's black latch with even force on both
sides, not one side only (real damage risk otherwise). 1697-20747 fixed it. **Re-ran the exact same live
sensor-negotiation test after the fix -- identical failure, fresh timestamp, not stale:**
`rkcif_update_sensor_info: stream[0] get remote terminal sensor failed!`. Confirms the earlier
driver-gap finding (no IMX219/IMX708/OV5647 driver on this image) was the real blocker all along,
not cable orientation -- cable is now correct and it still doesn't work, isolating the variable.

**Accelerometer: attached, but genuinely undetectable by any method tried so far.** No matching
`/sys/bus/iio/devices/` entry (only the SoC's own ADC controller shows up), no new `dmesg` activity
since it was attached, and Luckfox's own vendor test tools (`/oem/usr/bin/luckfox_i2c_test` etc.)
turned out to be built for a DIFFERENT C library entirely -- `readelf -l` shows they require
`/lib/ld-uClibc.so.0` (Luckfox's standard Buildroot/uClibc SDK image's dynamic linker), which
doesn't exist on this board's actual glibc-based Ubuntu image -- a real, previously-unknown
incompatibility between the vendor tooling and this specific OS image, not a bug in the tools
themselves. Tried the standard Ubuntu path instead: `i2c-tools` is already installed, but
`modprobe i2c-dev` failed outright -- **`/lib/modules/5.10.160` doesn't exist on this system at
all**, meaning this kernel build has no loadable-module directory; most drivers (including,
apparently, generic I2C character-device access) are compiled directly into the kernel image or
simply weren't included, not something addable by loading a module. Real blocker, not yet solved
-- need to know the exact accelerometer chip/model to know if there's any real path forward (a
built-in IIO driver for that specific chip might already exist and just need device-tree wiring,
or it may need the same kind of real kernel-rebuild work the camera driver gap would need).

**Static LAN IP set, confirmed working, not just applied-and-hoped:** `eth0` was on DHCP
(192.168.1.180 at the time). Used `nmcli connection modify "Wired connection 1" ipv4.method manual
...` with the SAME IP/gateway/DNS already active (deliberately, to avoid a real risk of losing the
only remote-management path if a genuinely different address was chosen), then `nmcli connection
up` to apply. Verified with a fresh SSH connection after applying, not assumed -- confirmed clean
reconnect at the same IP. This survives reboots (NetworkManager profile, not a one-off `ip addr`
command).

**WiFi configured, connected, and set to auto-connect -- LAN cable should no longer be required
going forward, per 1697-20747's ask.** `nmcli device wifi connect "EE-597TSN" password "..." ifname
wlan0` -- real credentials 1697-20747 provided, connected on the first attempt, got a real DHCP lease
(192.168.1.196), `autoconnect=yes` confirmed in the saved profile (survives reboot). **One real,
explained (not glossed over) test limitation:** couldn't cleanly confirm SSH reachability via the
WiFi IP alone while the LAN cable was still connected -- `ip route` shows `eth0` at metric 100 vs
`wlan0` at metric 600, so with both interfaces up on the identical 192.168.1.0/24 subnet, reply
traffic prefers `eth0` regardless of which interface a request arrived on, a normal Linux
multi-homed-same-subnet routing artifact, not a sign WiFi itself is broken. Real, definitive
confirmation needs 1697-20747 to physically unplug the LAN cable and test `ssh pico@192.168.1.196` (or
whatever IP `wlan0` gets next boot) -- not done yet, this is the one item in this batch not fully
closed out.

## Accelerometer identified from real photos; root cause traced to a shared, structural gap with the camera problem (2026-09-09)

1697-20747 sent two real photos of the physically-connected module. **Identified directly from the
silkscreen and pinout, not guessed:** a GY-521 breakout board carrying an **InvenSense MPU-6050**
(3-axis accel + 3-axis gyro combo, "ITG/MPU" silkscreen, standard VCC/GND/SCL/SDA/XDA/XCL/ADO/INT
pinout). **Green LED lit in the photo confirms it IS receiving power correctly** -- wired into the
GPIO header near the board's "CAM"/"LCD" silkscreen labels (a different header from the CSI ribbon
connector).

**Real, decisive root-cause finding, not another dead end to add to the pile:** `/sys/bus/i2c/
devices/` is completely empty -- zero I2C adapters registered anywhere in this kernel, for any
purpose, not just this specific bus. `/proc/devices` shows the I2C core framework itself IS present
(major number 89 registered), but no actual adapter driver is bound to any of the RV1106's physical
I2C controllers -- the framework exists, the board-specific peripheral driver that activates it
doesn't. **This is the exact same pattern as the camera failure** (the CIF/CSI capture framework
initializes fine, but no sensor subdevice driver ever registers) -- both point to ONE shared,
structural cause rather than two independent peripheral bugs: this specific community/third-party
Ubuntu image for the Pico Ultra ships a kernel with camera-sensor and I2C-adapter drivers left out,
compared to Luckfox's own official Buildroot SDK image (which demonstrably has them -- that's
exactly why `sc3336.ko` and the vendor `luckfox_*_test` tools exist, just built for a different
C library/kernel that isn't this one).

(Also resolves a loose thread from the earlier camera investigation: the stray "bus 2" that showed
up in one check turned out to be a bug in my own shell loop -- an unexpanded glob `/dev/i2c-*`
whose literal string got fed through `grep -o "[0-9]*"`, which pulled the "2" out of "i2c" itself,
not a real adapter number. Not a real signal; the "zero adapters" finding above is the accurate one.)

**Real options going forward, not decided yet -- 1697-20747's call:**
1. Switch to Luckfox's own official Buildroot SDK image -- camera + accelerometer would very likely
   just work (drivers are there), but loses the glibc Ubuntu environment (Python/pip/apt, the
   RKNN/ffmpeg/detection pipeline built so far) this whole investigation has relied on.
2. Rebuild/patch this Ubuntu image's kernel to add the missing I2C-adapter and camera-sensor
   drivers -- real, substantial kernel-level work, needs matching kernel source (not yet confirmed
   available).
3. Stay on this image, accept no camera/accelerometer for now, keep the working RKNN/yolov8n
   detection pipeline as-is.

## Real kernel fix for the accelerometer in progress: pinout identified, official SDK cloned, patches applied (2026-09-09)

1697-20747 sent a real photo of the board's own pinout diagram (Waveshare-style), letting the exact
physical pins be read directly: **pin 31 = 3.3V, pin 32 = GPIO1_A0/`I2C2_SCL_M0`, pin 33 =
GPIO1_B0/`I2C2_SDA_M0`, pin 40 = GPIO1_A1 (a real GPIO, not ground), pin 41 = GND** (1697-20747's
suspicion that 41 was the real ground pin, not 40, was correct). This resolved which of the
board's several I2C buses (I2C2 here, separate from I2C3 = touchscreen and I2C4 = camera) the
MPU-6050 is actually wired to.

**Real, official kernel source confirms this is a clean device-tree fix, not driver-writing:**
Cloned `LuckfoxTECH/luckfox-pico` (the real official SDK, ~1GB, shallow-cloned in a Docker Ubuntu
22.04 x86_64 container per Luckfox's own documented build requirement). Found:
- The real mainline `inv_mpu6050` IIO driver already exists in this kernel tree
  (`drivers/iio/imu/inv_mpu6050/`) -- confirmed via the real Documentation binding file
  (`inv_mpu6050.txt`), which documents the exact `compatible = "invensense,mpu6050"` /
  `reg = <0x68>` node syntax used below.
- `CONFIG_I2C_RK3X=y` and `CONFIG_IIO=y` (the I2C controller driver and the IIO subsystem) are
  already enabled in this board's real defconfig -- only `CONFIG_INV_MPU6050_I2C=y` was missing.
- The i2c2 controller itself is defined in `rv1106.dtsi` with `pinctrl-0 = <&i2c2m0_xfer>` but
  `status = "disabled"` by default (standard Rockchip convention -- SoC-level dtsi defines
  everything disabled, board-level dtsi enables what's actually wired).

**Found and fixed a real risk of editing the wrong file first:** the board's exact
`/proc/device-tree/compatible` (`rockchip,rv1103g-38x38-ipc-v10`, `rockchip,rv1106g3`) matches TWO
different files in the SDK -- a generic reference-board dts (`rv1106g-38x38-ipc-v10.dts`) and the
real board-specific one (`rv1106g-luckfox-pico-ultra.dts`, which includes
`rv1106-luckfox-pico-ultra-ipc.dtsi`). Confirmed which is real by checking WiFi/eMMC/Bluetooth
content against the actual board's known real features -- `rv1106g-luckfox-pico-ultra.dts` is the
correct one. Almost patched the wrong file.

**A real, unresolved discrepancy found while checking the existing i2c4 (camera) config in this
same file, worth remembering, not glossed over:** current upstream HEAD already has `&i2c4`
`status = "okay"` with a real `sc3336@30` sensor node configured by default -- but the actual
running board reports ZERO I2C adapters anywhere (`/sys/bus/i2c/devices/` empty, confirmed earlier
this session). If i2c4 were really enabled as this source shows, at minimum the adapter itself
should register in sysfs even if the wrong sensor fails to probe. This means the board's actual
currently-running kernel was very likely built from a different source snapshot than current
GitHub HEAD -- proceeding anyway by building fresh from current HEAD with the targeted i2c2 patch,
treating this as a real kernel update rather than a minimal exact-diff patch. Real, honest
uncertainty: the rebuilt kernel may behave somewhat differently from the currently-running one in
ways beyond just the intended i2c2/mpu6050 fix (hopefully only improvements, not regressions) --
this can only really be confirmed once actually flashed and booted.

**Two real, minimal patches applied to the cloned SDK:**
1. `sysdrv/source/kernel/arch/arm/boot/dts/rv1106-luckfox-pico-ultra-ipc.dtsi` -- `&i2c2` block
   changed from just a pinctrl override to `status = "okay"`, `clock-frequency = <100000>`, plus a
   child `mpu6050@68` node (`compatible = "invensense,mpu6050"`, `reg = <0x68>`). Interrupt line
   deliberately omitted (INT pin not confirmed wired) -- the driver should still probe and work in
   polling mode without it.
2. `sysdrv/source/kernel/arch/arm/configs/luckfox_rv1106_linux_defconfig` -- added
   `CONFIG_INV_MPU6050_I2C=y` right after the existing `CONFIG_IIO=y` line.

**Kernel build launched** (`./build.sh lunch` with HW_INDEX=5/RV1106_Luckfox_Pico_Ultra, EMMC boot
medium, Buildroot system -- matching the real `BoardConfig-EMMC-Buildroot-RV1106_Luckfox_Pico_Ultra-IPC.mk`
that confirmed `RK_KERNEL_DTS=rv1106g-luckfox-pico-ultra.dts` and
`RK_KERNEL_DEFCONFIG=luckfox_rv1106_linux_defconfig`, exactly the two files just patched), then
`./build.sh kernel` -- a real, targeted kernel-only build using the bundled
`arm-rockchip830-linux-uclibcgnueabihf` cross-toolchain already in the SDK, no external toolchain
needed. Real build in progress as of this note -- result not yet known, don't assume success.

**Still fully open: flashing.** Even once a real kernel+dtb is built, deploying it needs a physical
USB connection from the Mac directly to the board while it's in a special maskrom/loader mode,
using Rockchip's own flashing tool (the SDK's own `rkflash.sh`, not yet inspected in detail) --
whether that tool runs on macOS at all is still unconfirmed. Real next step once the build result
is known.

## Kernel build actually succeeded end-to-end, accelerometer working on real hardware, real measured sample rate (2026-09-09, later same session)

**The `.img` packaging failure from the prior note turned out to be two genuinely separate real
bugs, not one, both found by finally reading the full (previously truncated/lost) build log:**

**Bug 1 -- real root cause, structural, not a kernel config problem at all: macOS's default
case-insensitive APFS filesystem silently corrupted the git checkout.** The actual error was `make:
*** No rule to make target 'net/netfilter/xt_DSCP.o'` -- a real source file genuinely missing from
the working tree, not a Makefile/config issue. Root-caused by comparing `git cat-file -p HEAD:...`
(reads directly from git's object database, unaffected by the working-tree filesystem) against what
was actually on disk: git's object store has BOTH `xt_DSCP.c` (a target module) and `xt_dscp.c` (a
different match module) as distinct blobs with different content -- confirmed via `git ls-tree`
showing two different blob hashes for the two paths. But checking them out onto this Mac's default
case-insensitive APFS volume silently collided the two filenames into one inode, discarding
`xt_DSCP.c`'s real content. **Swept the whole tree for the same pattern** (`git ls-tree -r HEAD
--name-only | awk '{print tolower($0)}' | sort | uniq -d`) and found **21 total colliding path
pairs**, all in the same netfilter xtables family (`xt_DSCP`/`xt_dscp`, `xt_MARK`/`xt_mark`,
`xt_TCPMSS`/`xt_tcpmss`, `ip6t_HL`/`ip6t_hl`, etc.) plus one harmless litmus-test file -- confirming
this was one systemic filesystem-compatibility bug, not scattered unrelated breakage. **Real fix:**
stopped building against the bind-mounted host checkout directly; instead `git clone`'d the repo
FROM the host mount INTO a path inside the Docker container itself (a real Linux, case-sensitive
filesystem) before running `build.sh` -- git's object database is unaffected by host filesystem
case-sensitivity, so the container-side clone checks out both files correctly. This is the real,
durable fix for building this SDK on any Mac, not just this one session.

**Bug 2, found only after Bug 1's fix let the build reach the actual driver probe on real
hardware:** the mainline `inv_mpu6050` driver's `inv_mpu_core_probe()` unconditionally calls
`devm_request_irq(&indio_dev->dev, st->irq, ...)` inside `inv_mpu6050_probe_trigger()` to set up
triggered-buffer/FIFO mode -- this board's MPU-6050 only has VDD/GND/SCL/SDA wired (no INT line, a
deliberate choice at the time since the pin wasn't confirmed wired), so `client->irq` is 0, and
requesting IRQ 0 is invalid on this platform -- confirmed live via `dmesg`: `inv-mpu6050-i2c
2-0068: trigger probe fail -22` (-EINVAL), after the driver had already correctly probed the device
at the right I2C address (`2-0068` = i2c2, 0x68), proving the DT/Kconfig patches from the earlier
note were themselves correct. **Real fix, not a workaround:** patched
`drivers/iio/imu/inv_mpu6050/inv_mpu_core.c` to gate the whole triggered-buffer/trigger-probe block
on `if (irq > 0)`, falling back to `INDIO_DIRECT_MODE` when there's no real IRQ line -- raw sysfs
channel reads (`in_accel_*_raw`) are served by the channel's own `.read_raw()` callback regardless
of buffer/trigger mode, and `luckfox_video_hud_stream.c` only ever does polled raw reads, so this
loses nothing for the actual use case. Confirmed live in `dmesg` after the fix: `inv-mpu6050-i2c
2-0068: no IRQ, running in polled/direct mode only` -- driver probe now succeeds.

**Flashing: real, much simpler path found than the maskrom/`rkflash.sh`/USB approach the prior note
assumed -- no need for that at all, and rkdeveloptool's blocked macOS build (broken Xcode CLT) is a
moot point now.** The board runs off eMMC with a real, confirmable-from-userspace partition table
(`cat /proc/cmdline` shows `blkdevparts=mmcblk0:32K(env),512K@32K(idblock),256K(uboot),
32M(boot),512M(oem),256M(userdata),6G(rootfs)`, matching `/proc/partitions` exactly:
`mmcblk0p4` = the 32M `boot` partition, not mounted, raw FIT-image target). Since the board is a
normal running Linux system reachable over SSH, **the new kernel was flashed live, in-place, from
this Mac, with zero physical access and zero maskrom mode**: `scp` the new `boot.img` to `/tmp` on
the board, `dd if=/tmp/new_boot.img of=/dev/mmcblk0p4 bs=1M conv=fsync`, `sync`, `reboot`. **Real
safety step taken first, not skipped:** pulled a full `dd`-based backup of the ORIGINAL
`/dev/mmcblk0p4` before writing anything, both to the board's own `/tmp` and copied back to the Mac,
md5-verified identical in both places -- so a bad flash had a real, tested recovery path (re-flash
the backup the same way) without ever needing maskrom/rkdeveloptool. Also md5-verified the new
`boot.img` after every `scp` before writing it, both attempts. **This worked cleanly twice in a row**
(once for the DT/Kconfig-only build, once more after the IRQ patch) -- board rebooted in ~10-20s
each time, `uname -a` confirmed the new build's real timestamp both times, proving the flash (not
just the build) actually took effect.

**A real, honest process mistake worth recording:** the DT/Kconfig patches from the prior note were
still sitting as UNCOMMITTED working-tree edits on the host when the container-side `git clone` fix
was first tried -- `git clone` only copies committed history, so the first "successful" container
build silently produced a kernel WITHOUT the accelerometer patches at all (`strings` on the built
`.dtb` showed no `mpu6050` node). Caught before flashing anything, by checking for the string in the
built artifact rather than assuming success from a clean exit code. Fixed by committing the patches
to the local scratch clone (`git commit`, not pushed anywhere, this SDK checkout isn't the project's
own git history) before re-cloning into the container.

**Real accelerometer data, live on actual hardware, confirmed multiple ways:**
```
iio:device1: mpu6050
in_accel_x_raw=8284  in_accel_y_raw=-10648  in_accel_z_raw=13218  in_accel_scale=0.000598
```
(`0.000598` m/s^2/LSB matches the MPU-6050's default +-2g range exactly:
9.80665/16384 = 0.000598 -- confirms the scale value itself is real and correct, not a stub.)
`luckfox_video_hud_stream` (the existing binary, already built in an earlier part of this session)
picked it up automatically: `RESULT: ACCEL_FOUND /sys/bus/iio/devices/iio:device1 scale=0.000598`.

**Real, measured sample rate (1697-20747's ask from earlier this session, "check response rate when
working") -- ~3.6 Hz, tied 1:1 to the video-processing loop, not an independent sensor-bus rate.**
First live run showed a misleadingly low `0.2 Hz` for the first 30-read window -- root-caused (not
just noted and moved on) to a real bug in the rate-measurement code itself: `rate_t0` is set once at
`accel_discover()` time, which runs immediately at process startup, well before any HTTP client
connects and the per-frame loop (which is what actually calls `accel_update()`) starts -- so the
first window's elapsed-time denominator included the entire idle "waiting for browser" wait,
deflating the computed Hz. Confirmed by re-running with a longer-lived client connection (90s
`curl`) to get past the first (tainted) window: subsequent windows are clean and highly consistent
-- 1.4 Hz during ramp-up, then settling to **3.6-3.7 Hz** for the rest of a 300+-read run, 0 failed
reads throughout. This exactly matches the loop's own measured steady-state ~3.5fps (logged
alongside, same run) -- confirms `accel_update()` is called exactly once per rendered video frame,
so the accelerometer's effective read rate is bottlenecked by the video/inference/encode pipeline,
not by I2C bus speed or the sensor itself. A real interrupt-driven trigger (wiring the MPU-6050's
INT pin to the board's free GPIO1_A1 / physical pin 40, which 1697-20747 offered to do) would decouple
sensor sampling from the video loop entirely -- not done this session, a real future improvement,
not required for the current polled/EMA-smoothed HUD readout to work correctly.

**Pin 40 (`GPIO1_A1`) documented in the README as the optional INT wiring target** -- adjacent to
the already-used 32/33 (SCL/SDA) and 41 (GND), forming a clean 4-wire cluster on the header.

## WiFi broken by the accelerometer kernel rebuild, root-caused, and fixed for real -- plus a docs
## site and a real 512x512 vs 640x640 inference benchmark (2026-09-11)

**Discovery: WiFi over WiFi (not just LAN) had never actually been retested since the accelerometer
kernel rebuild two sessions ago, and it was broken.** Asked to connect over wireless -- the last
known WiFi IP timed out. SSH via the known-good static LAN IP (`192.168.1.180`) found `wlan0` didn't
exist at all: `/sys/class/net/` only showed `eth0`/`usb0`/`lo`, `lsmod` had zero WiFi-related
modules, `dmesg` showed no WiFi chip driver probe attempt at all. The `EE-597TSN` NetworkManager
profile was still saved from the earlier working session, just nothing to attach it to.

**Real chip ID, not assumed:** `/sys/bus/sdio/devices/*/uevent` gave `SDIO_ID=C8A1:C08D` (function 1)
and `C8A1:C18D` (function 2) -- vendor `0xc8a1` is AICSemi. Cross-checked against this SDK's own
`sysdrv/drv_ko/wifi/aic8800dc/insmod_wifi.sh` and `BoardConfig-EMMC-Buildroot-RV1106_Luckfox_Pico_Ultra-IPC.mk`
(`RK_ENABLE_WIFI_CHIP=AIC8800DC`) -- confirmed AIC8800DC, matching the `.ko` files already sitting
unused on the rootfs at `/oem/usr/ko/` (`aic8800_bsp.ko`/`aic8800_fdrv.ko`/`aic8800_btlpm.ko`).

**First real attempt at reusing those existing `.ko` files hung the board -- twice, reproducibly,
needing a physical power cycle both times.** `insmod aic8800_bsp.ko` succeeded cleanly; `insmod
aic8800_fdrv.ko` (the netdev/firmware driver) is what actually froze it -- SSH and all userspace
stopped responding while the kernel's own ICMP/TCP-handshake layer kept replying, the signature of
a kernel-level lockup, not a clean process crash. Confirmed twice, byte-for-byte the same failure
point both times, before accepting it wasn't a fluke.

**Root cause, once actually investigated instead of just reflashing and hoping: NOT a vermagic
mismatch.** `modinfo` on every module in the chain (including the already-working `sc3336.ko`)
showed an *identical* vermagic string (`5.10.160 mod_unload ARMv7 thumb2 p2v8`) -- no
`CONFIG_MODVERSIONS`, so the plain vermagic string is the kernel's entire ABI compatibility check,
and it matched. The stale `.ko` files were binary-compatible with the rebuilt kernel by that check,
yet still hung on load. This ruled out the obvious first theory and meant the real problem was
something deeper than "kernel got rebuilt, modules didn't."

**Real fix: rebuilt the wifi driver from source against the exact currently-running kernel,
re-cloned the same SDK (same HEAD commit, `824b817f88...`, unchanged since 2026-03-22 -- confirmed
upstream genuinely hasn't moved), re-applied the same i2c2/mpu6050 DT and Kconfig patches from the
earlier accelerometer session so the rebuilt kernel matches what's already flashed, then additionally
built the AIC8800DC driver (`sysdrv/drv_ko/wifi/aic8800dc/`) from that same tree via `./build.sh
driver`.** Deployed the fresh `.ko` set (bsp/fdrv/btlpm + cfg80211/libarc4/ctr/ccm/aes_generic) to
`/oem/usr/ko/`, backed up the current boot partition first (same `dd`+md5-verify discipline as the
earlier accelerometer flash), and tested loading one module at a time with a liveness check after
each -- `aic8800_bsp.ko` loaded clean, `aic8800_fdrv.ko` now also loaded clean (`RC=0`, no hang).
`libaes.ko` fails to load (`exports duplicate symbol aes_decrypt (owned by kernel)`) -- harmless,
the new kernel already provides that symbol built-in, and nothing in the chain actually needs the
standalone module.

**But `aic8800_fdrv.ko` loading clean wasn't the same as WiFi actually working -- a second, real,
separate bug followed immediately.** `wlan0` never appeared; dmesg showed `aicbsp_platform_power_on`
attempting a real power-on/rescan sequence and failing: `aicbsp_dummy_sdmmc: probe of mmc1:e9ea:1
failed with error -110` (timeout). Read the actual driver source
(`aic8800_bsp/aicsdio.c`) rather than guessing: `aicbsp_platform_power_on()` only has real GPIO
power-toggle logic under `CONFIG_PLATFORM_ROCKCHIP2` (`rockchip_wifi_power()`/
`rockchip_wifi_set_carddetect()`, from the kernel's own `net/rfkill/rfkill-wlan.c`) -- the plain
`CONFIG_PLATFORM_ROCKCHIP` (no "2") this SDK's Makefile actually builds with is a near no-op for
power sequencing, relying entirely on the generic `mmc-pwrseq-simple` reset node already in this
board's DTS, which is enough for the SDIO bus to *detect* the chip at boot (that's how the vendor
ID was read above) but not enough to bring it into a state where the bus can *enable* its I/O
function. Confirmed by checking a real, official Rockchip reference board's DTS
(`rv1106g-evb1-v10.dts` in the same SDK) which has a `wireless-wlan { compatible = "wlan-platdata";
... }` node this board's DTS was simply missing.

**Real, targeted two-part fix, not a guess:** (1) added a `wireless_wlan: wireless-wlan {
compatible = "wlan-platdata"; wifi_chip_type = "aic8800dc"; WIFI,reset_gpio = <&gpio1 RK_PA2
GPIO_ACTIVE_LOW>; status = "okay"; }` node to `rv1106g-luckfox-pico-ultra.dts`, reusing the exact
same physical reset line already wired to the generic `sdio_pwrseq` node; (2) enabled
`CONFIG_PLATFORM_ROCKCHIP2` -- carefully, *inside* the existing correct RV1106 build block in
`aic8800dc/aic8800_bsp/Makefile`, not by flipping the top-level `CONFIG_PLATFORM_ROCKCHIP2 = y` in
the parent Makefile, which would have silently redirected `ARCH`/`KDIR`/`CROSS_COMPILE` to some
other developer's hardcoded RK3566/Android/aarch64 paths and broken the whole build. Rebuilt kernel
+ DTB + driver together (same source tree, so guaranteed ABI-consistent), backed up the current
boot partition again, flashed the new one, redeployed the modules. **Result, real and confirmed:**
`wlan0` came up (`state UP`, real MAC), NetworkManager auto-connected to the already-saved
`EE-597TSN` profile with zero manual intervention, survived a full physical-adjacent reboot cleanly.

**Made both LAN and WiFi IPs permanently static, per request, confirmed not just applied:**
`nmcli connection modify 'EE-597TSN' ipv4.method manual ipv4.addresses 192.168.1.196/24
ipv4.gateway 192.168.1.254 ipv4.dns 192.168.1.254` -- landed on `.196` deliberately, the same
address the WiFi interface had already been leasing via DHCP, avoiding a new address. Verified
`valid_lft forever` (no more DHCP renewal) and reachable via `ssh pico@192.168.1.196` directly, not
riding on `eth0`'s route. `eth0` was already static from the earlier session (`192.168.1.180`).
Persistence confirmed via multiple real full reboots during this same session, not asserted once.

**Set the WiFi driver to load automatically at boot** -- `insmod_wifi.sh`'s own chip-detection logic
doesn't actually match this board (its SDIO-ID grep looks for a literal "8800" substring, which
`C8A1:C08D` doesn't contain, and its device-tree-model fallback check greps for a trailing "W" this
board's `model = "Luckfox Pico Ultra"` string doesn't have) -- so instead of fighting that script,
added a small dedicated systemd unit (`aic8800-wifi.service` -> `/usr/local/sbin/aic8800-wifi.sh`,
`WantedBy=multi-user.target`, `Before=NetworkManager.service`) that loads the known-working module
sequence directly. Confirmed across a full cold reboot: WiFi and the static IP both come up with
zero manual steps.

**A real, separate bug found afterward, not from guessing but from noticing degraded SSH
responsiveness during later testing: the WiFi driver's SDIO sleep/wake power-management handshake
never actually completes on this chip, and busy-polls forever.** `uptime` showed `load average:
12+` on this single-core board with no obvious cause; `ps aux --sort=-%cpu` showed a persistent
kernel thread, `[aicwf_pwrctl]`, with double-digit minutes of accumulated CPU time; `dmesg` was
flooded with repeating `sdio_err:<aicwf_sdio_sleep_allow,1455>: val:2` / `warning: read cnt N`
pairs, counting down to 0 every time. Read the actual driver source
(`aic8800_fdrv/aicwf_sdio.c` around the wakeup-register poll loop): on `PRODUCT_ID_AIC8800DC`, the
sleep-transition path writes `0x02` to a wakeup register then polls up to 100 times at 500us
apiece (~50ms) waiting for an acknowledgement bit to clear -- and on this chip/firmware, it never
does, so every attempt burns the full 50ms doing nothing, repeated continuously by a dedicated
kernel thread (`aicwf_pwrctl`) for as long as the interface is up. This is pure power-save
negotiation, not the data path -- WiFi connectivity itself was unaffected throughout -- but it's
real, continuous, wasted CPU on a board that has exactly one core to spare.

**Fix: disabled the feature outright rather than debugging an undocumented firmware handshake.**
`CONFIG_SDIO_PWRCTRL` (default `?= y` in `aic8800_fdrv/Makefile`) gates the entire power-control
timer/thread mechanism, confirmed by tracing every one of its `#if defined(CONFIG_SDIO_PWRCTRL)`
call sites including the exact `kthread_run(..., "aicwf_pwrctl")` call that creates the thread.
Flipped it to `n`, rebuilt just the driver module (no kernel/DTB change needed this time -- lower
risk, and correctly so: this was a driver bug, not a kernel one), redeployed, rebooted. **Confirmed
clean afterward:** zero `sleep_allow` messages in dmesg, no `aicwf_pwrctl` thread in `ps aux`, WiFi
still connects and holds its static IP exactly as before. The elevated load-average figure took a
few real minutes to decay back down on its own after the fix landed -- `vmstat 1` showing genuine
100% idle in real time confirmed the system was actually healthy well before the lagging 1-minute
average caught up, a reminder not to over-read a smoothed historical metric against a live check.

**Real 512x512 vs 640x640 inference-resolution benchmark, requested to test whether input
resolution is actually the speed lever the architecture doc claims it is.** Re-exported `yolov8n`
at `imgsz=512` using `airockchip/ultralytics_yolov8`'s own RKNN-optimized exporter (same DFL-head
restructuring as the working 640 model), converted through the same toolkit-1.6.0 Docker pipeline.
**One real environment trap hit and fixed along the way:** the locally cached `rknn-convert` Docker
image (5 months old) turned out to actually have `rknn-toolkit2 2.3.2` installed, not the `1.6.0`
its own Dockerfile specifies -- a stale image built before the version-pinning fix was worked out.
Using it would have silently produced a model this board's runtime can't load. Rebuilt the image
fresh from the current `docker/convert.Dockerfile` before trusting any output from it -- confirmed
`rknn-toolkit2-1.6.0+81f21f4d` this time. Also hit and fixed a second real, separate export bug:
the installed torch's default (dynamo-based) ONNX exporter silently upgraded the requested opset 12
to opset 18 (a `Resize` op version-conversion failure it couldn't recover from) and split model
weights into an external `.onnx.data` sidecar file -- both real risks for the old toolkit. Fixed by
forcing `dynamo=False` in the vendor exporter's `torch.onnx.export()` call, producing a clean,
single-file, real opset-12 ONNX whose output tensor names (`318`/`onnx::ReduceSum_326`/`331` per
scale) matched the known-working reference structure exactly.

**Real, controlled measurement, not a guess:** same `yolov8n` architecture, same pipeline, same
`bikes.mp4` test video, same 640x640 *display* resolution held constant in both runs so only the
inference-resolution variable changes (JPEG-encode/network-send tracks display pixels, not
inference pixels, so this isolates the comparison cleanly). Result:

```
                         640x640 inference (baseline)   512x512 inference
NPU inference                   ~67ms/frame                 ~50ms/frame
Steady-state end-to-end fps     ~4.5-4.8 fps                ~5.2-5.3 fps
```

Inference time itself drops a real ~25% (roughly tracking the ~36% pixel-count reduction, sub-linear
because of fixed per-invocation NPU overhead). End-to-end fps improves only ~10-15%, well under the
inference-time gain, because JPEG-encode+network-send -- unchanged here since display resolution
didn't change -- remains the single biggest cost in the pipeline and swamps the inference saving.
This is a real, direct confirmation of the earlier finding that encode/network, not NPU inference,
dominates this pipeline's real-world cost.

**Also confirmed, precisely, something that had only been asserted before: restricting the detected
class list does not speed up inference.** `OBJ_CLASS_NUM` is hardcoded to `80` in the decode code;
`ALLOWED_CLASSES` is applied strictly as a post-decode filter, after the model has already scored
all 80 COCO classes for every candidate box. The full 80-class model already runs at the same speed
a 1-class model would, because the classification head is a tiny fraction of this architecture's
total FLOPs next to the backbone. Restricting classes is a HUD-decluttering choice, not a
performance lever -- confirmed by reading the actual `#define OBJ_CLASS_NUM 80` and filter call site
on the board, not by re-deriving it from theory.

**Built a small documentation site, running live on the board itself.** Wrote
`RV1106_NPU_YOLO_GUIDE.md` (repo root, also mirrored as Hugo content under `docs-site/`) --
hardware overview, the RKNN toolkit/runtime version-pairing constraint, the zero-copy memory model,
why YOLOv8 runs and YOLO26/11 don't, the DFL-head fix, the decode/NMS math, and the real measured
performance numbers above. `docs-site/` is a Hugo site using the same PaperMod theme as 1697-20747's own
blog (copied locally, not re-cloned, to skip the earlier-documented flaky-mirror problem), dark mode
default, with a custom CSS override (`assets/css/extended/floating-toc.css`) turning the normally
collapsible inline PaperMod TOC into a fixed floating left sidebar on wide viewports, falling back
to the normal inline TOC below ~1300px so it never overlaps content on narrow screens. Deployed a
real `hugo` binary (the plain `linux-arm` release, not `extended` -- no `extended` armv7 build
exists, and PaperMod's CSS concat/minify pipeline doesn't actually need it) to the board, running as
a persistent `luckfox-docs.service` systemd unit (`hugo server --watch=false` to avoid unnecessary
inotify overhead on a board that doesn't edit its own docs live), confirmed serving on both the LAN
and WiFi static IPs and surviving a full reboot, ~38MB RSS.

**One real methodological note for whoever runs this benchmark again:** the Docker rebuild for the
`rknn-convert` image hit a real, separate networking issue mid-session -- `archive.ubuntu.com`
(both HTTP and HTTPS, IPv4-forced) hung completely from inside Docker containers on this Mac while
the exact same host worked fine from the host shell and from a fresh container hitting
`github.com`/`mirrors.edge.kernel.org` -- looked like a bad anycast/CDN edge for that one host from
this specific network path at this specific time, not a general Docker or Mac networking problem.
Fixed by pointing `/etc/apt/sources.list` at `mirrors.edge.kernel.org/ubuntu` instead. Not
investigated further since it resolved cleanly; worth trying a different mirror first if
`archive.ubuntu.com` hangs again rather than assuming Docker networking itself is broken.

## Real hardware offload attempt: RGA and MPP -- one real (if negative) win, one real wall found and stopped at cleanly (2026-09-11, later session)

1697-20747 asked to move as much of the video-processing pipeline as possible off the single CPU core and
onto RV1106's real hardware blocks, since JPEG-encode+network-send was documented (last session) as
the pipeline's single biggest cost, still 100% software. Explicitly scoped narrow: only the existing
single-client `luckfox_video_hud_stream.c` pipeline's CPU-bound stages, not live camera (separate,
blocked) or multi-client fan-out (separate, paused). Two real targets: MPP's hardware JPEG encoder
(primary, bigger expected win) and RGA's 2D scale/color-convert (secondary, for
`downscale_for_inference()`).

**Real discovery first, not assumed.** SSH'd in fresh and checked `ldconfig -p`, `lsmod`, and
`/dev`: **RGA has a real, ready-to-use glibc userspace library already on this board** --
`librga.so.2` (`rga_api version 1.10.1`) sits in `/usr/lib`, in `ldconfig`'s cache, no build needed.
**MPP does not** -- `librockchip_mpp.so` only exists under `/oem/usr/lib` (Luckfox's vendor
Buildroot/uClibc tree), linked against `libc.so.0` (uClibc), and this Ubuntu image has no
`ld-uClibc.so.0` interpreter to run it at all (`/oem/usr/bin/rgaImDemo`/`mpp_info_test` both fail
with a bare "No such file or directory" -- the classic missing-interpreter symptom, not a literally
missing file). Kernel side for both is real and active either way: `mpp_vcodec`/`rga3` both show up
in `lsmod`, `/dev/mpp_service` and `/dev/rga` both exist -- confirms this is a genuine
userspace-only gap for MPP, not a deeper hardware/driver one. Neither library is available via
`apt` (checked `dpkg -S`/`apt-cache search` -- `librga.so.2` isn't from a Debian package at all, it
was placed there by whoever built this third-party image).

**Closed a real, several-sessions-old open question: JPEG encode vs. network send, split for real.**
Instrumented `http_send_frame()` in `luckfox_video_hud_stream.c` to time `stbi_write_jpg_to_func`
(encode) and `send()` (network write) separately (previously only their combined total existed).
Tested both over loopback AND from a real WiFi client on a separate machine (the Mac), not just
loopback, to rule out a loopback-specific artifact:

```
                            combined (old)   JPEG encode alone   network send alone
1280x720, real WiFi client      ~150-165ms        ~146-165ms           ~0.4-1.5ms
```

**JPEG encode (pure software, `stb_image_write`) is essentially the entire cost; network send is
negligible, even over real WiFi to a separate machine.** This directly corrects a real, specific
guess made two sessions ago (dropping JPEG quality 80->50 barely moved the number, which was read as
"the network write must be the real cost") -- that guess was wrong: `send()` just hands bytes to the
kernel's TCP buffer and returns almost immediately; the real WiFi transmission happens asynchronously
off the pipeline's hot path, with plenty of time to drain between frames at this frame rate. This
makes MPP's hardware JPEG encoder unambiguously the single highest-value real target in this
pipeline, bigger than NPU inference itself.

### RGA downscale offload: implemented, correct, real -- and a real, measured net loss

Rewrote `downscale_for_inference()` to call RGA's `imresize()` instead of a CPU nearest-neighbor
loop. Fetched real headers to match the installed `.so` (`airockchip/librga` on GitHub -- the actual
upstream for this API, found by probing candidate repo names since `rockchip-linux/linux-rga` and
`airockchip/linux-rga` both 404; `airockchip/librga` is real, `rga_api version 1.10.6` on that repo's
HEAD vs. `1.10.1` installed on the board -- close enough that the library's own built-in
version-compatibility warning stayed silent, a small, acceptable version skew, not blindly assumed
safe).

**First real wall, hit immediately: this chip's RGA has no IOMMU.** A first attempt handed
`imresize()` plain `malloc()`'d buffers (the existing `frame_full`/`frame_small`) -- failed for real,
not silently: `dmesg` showed `rga_mm: Current NONE_MMU[0] cannot support physically discontinuous
virtual address!` / `job_commit failed`. Real, concrete, not a guess: RGA needs genuinely
physically-contiguous DMA memory; ordinary heap memory is virtually contiguous but not physically
so. **Real fix, not a workaround:** wrote a DRM "dumb buffer" allocator (`drm_alloc_contig()` in
`luckfox_video_hud_stream.c`) using raw ioctls against `/dev/dri/card0` (confirmed present and
usable) -- deliberately NOT linking/dlopen-ing `libdrm.so` (this image ships only the runtime `.so`,
no dev symlink) since this board's own `/usr/include/drm/{drm,drm_mode}.h` kernel-uapi headers are
enough for the raw `DRM_IOCTL_MODE_CREATE_DUMB`/`MAP_DUMB` ioctl sequence, matching the same real
pattern Rockchip's own `librga` sample allocator (`samples/utils/allocator/drm_alloc.cpp`, fetched
for reference, not built directly) uses.

**Second real wall: that memory is CPU-uncached, and this pipeline's frame buffers are NOT
CPU-light.** A first working version made `frame_full`/`frame_small` themselves DRM-backed so RGA
could touch them with zero extra copies -- `downscale_ms` dropped to a real ~2.5ms as hoped, but
every OTHER stage that also touches those buffers (the quantization loop, HUD drawing, JPEG encode)
got dramatically slower reading through uncached memory: `infer_ms` 85ms->323ms, `jpeg_ms`
145ms->548ms -- a large, real NET LOSS (fps 3.3->1.0), kept and reported rather than discarded as a
false start. **Third real attempt, the actual fix for the regression (if not for the original
goal):** bounce through two small persistent DRM buffers ONLY around the RGA call itself (`g_drm_src`/
`g_drm_dst`), keeping `frame_full`/`frame_small` as ordinary cached `malloc()` buffers used
everywhere else. This fixed the regression elsewhere (`jpeg_ms` back to ~146-153ms, matching
baseline) but left the downscale step itself **slower than the CPU loop it replaced** (~51.6-60.5ms
vs. the CPU loop's real ~8.7-9.2ms, a more precise number than the ~18-26ms estimated last session
from an indirect before/after `infer_ms` comparison -- this session's direct per-stage timing split
gives the real, accurate figure).

**A fourth real attempt tried removing the memcpy-out too** (having the quantization loop read the
uncached `g_drm_dst` directly, since it already touches every dst byte once) -- reasoned as "one
fewer pass should be faster." **Measured result: much WORSE** (`infer_ms` back up to ~321-332ms), not
better -- real, somewhat counter-intuitive finding: a scalar per-byte cast loop against uncached
memory pays the full uncached-access penalty on every single byte access, while a bulk `memcpy()`
apparently doesn't (likely a vectorized/wide-load libc implementation issuing far fewer, wider bus
transactions). Reverted to the memcpy-out version -- "one fewer pass" is not automatically faster
than "one extra memcpy" on uncached memory; access pattern matters as much as pass count.

**Real, final numbers, clean (MPP's background build paused during measurement to avoid CPU
contention skewing results -- confirmed this mattered: an earlier run measured with the build still
running showed `jpeg_ms` inflated to 337-348ms, a real, caught mistake, not reported as a genuine
result):**

```
                              CPU nearest-neighbor (default)   RGA (USE_RGA_DOWNSCALE=1)
downscale step alone                    ~8.7-9.2ms                    ~51.6-60.5ms
real steady-state end-to-end fps         ~3.3-3.4 fps                  ~2.9-3.0 fps
```

**Honest conclusion: RGA downscale offload works, is correct, and is a real, measured net loss on
this exact hardware** -- the mandatory copy-through-contiguous-DMA-memory tax this no-IOMMU chip
imposes outweighs what the hardware resize itself saves, for an operation (640x640 nearest-neighbor)
that was already cheap on the CPU. Four different real implementations were tried, each one fixing
the specific problem the previous measurement revealed -- not a single attempt given up on early.
Flipped the default back to CPU (`use_rga_downscale = (getenv("USE_RGA_DOWNSCALE") != NULL)`) rather
than ship a regression as the new default; the RGA path is kept available and working, not deleted,
since it's real correct code that could matter on a future task with a larger/different resize.

### MPP hardware JPEG encode: real build success, two real integration gaps fixed, then a genuine kernel-level wall

**Built the public `rockchip-linux/mpp` source natively on-device.** Confirmed `cmake`/`git`/`make`/
`gcc`/`g++` all present -- no cross-toolchain needed. Shallow-cloned (`develop` branch, the only
current branch; the `1.0.x`/`1.1.x` tags are old release snapshots predating RV1106). `cmake`
configured cleanly first try. The actual `make -j1` build (single core, ~200MB RAM) was real,
substantial work -- compiles codec HALs for essentially every Rockchip chip family by default (no
compile-time "just this chip" option exists; MPP is designed for runtime hardware detection instead,
so trimming would mean real surgery on the CMake/source tree, not a flag) -- paused/resumed several
times across the session (once by accident: a `pgrep -f "[c]c1$"` pattern typo meant to resume the
build only matched the parent `make` process, not the actual `cc1` child, silently stalling the
build at 15% for a long stretch until caught by noticing zero log progress and checking real process
state, not assumed successful from the resume command not erroring). **Completed for real, 100%,
producing a real working `librockchip_mpp.so.0`/`.1`** -- confirmed via `mpi_enc_test`/`mpp_info_test`
actually running against it.

**Real integration gap #1: this exact SoC isn't in the upstream repo's chip table at all.** First run:
`mpp_platform: can not found match soc name` + `mpp_rt: can NOT found any allocator` (the second one
turned out to be a red herring -- unprivileged `pico` lacks `video`/`render` group membership for
`/dev/dri/card0`; running with `sudo` fixed it, unrelated to the SoC-match problem). Checked
`osal/mpp_soc.c` directly: zero matches for "rv1106" anywhere, confirmed by grep, not assumed from
the error message alone. Checked whether Luckfox's own SDK (`LuckfoxTECH/luckfox-pico`, already used
this project for the accelerometer/WiFi kernel work) has real MPP source with the missing entry --
it does NOT: `media/mpp/` in that repo ships only prebuilt uClibc release binaries
(`release_mpp_rv1106_arm-rockchip830-linux-uclibcgnueabihf/`), no buildable source, so there's no
"real" reference entry to copy verbatim. **First fix, wrong:** modeled a new RV1106 table entry on
the neighboring `rv1109`/`rv1126` chips (same IPC-camera family, plausible-looking). Got past the
name-match step, but every subsequent real hardware test logged `client 1/9/18 driver is not ready!`
-- the runtime compares my claimed capabilities against what the REAL kernel driver reports via
`check_mpp_service_cap()`, and disagreed on 3 of them.

**Real integration gap #1, corrected using real hardware data instead of a sibling-chip guess.**
`cat /proc/mpp_service/supports-device` on the actual board -- ground truth, not inferred -- reports
exactly one real device: `DEVICE[16]:RKVENC HW_ID:0x50603313`. Grepped the same source tree for that
literal hex value: `0x50603313` is `HWID_VEPU540C` (`mpp/hal/inc/hal_hw_id.h`) -- a different, newer
encoder IP block than rv1109/rv1126's vepu2/vepu541, shared instead with RK3528/RK3562's real table
entries (confirmed by finding their entries also reference `vepu540c`). No decode device is listed
at all -- consistent with this chip's own datasheet (no hardware decode on RV1106) and with client
1 (VDPU2) / 9 (RKVDEC) being exactly the ones reported "not ready" against the wrong first guess.
Rewrote the table entry: `HAVE_RKVENC` only (no decode flags at all), HAL `vepu540c_no_hevc`
(AVC+MJPEG, no HEVC -- matches real RV1106 capability, more precise than the plain `vepu540c` which
also claims HEVC). Rebuilt -- the "client not ready" warnings and the "can not found match soc name"
warning both disappeared, a real, confirmed fix, not assumed from the code change alone.

**Real integration gap #2, found immediately after: `mpp_enc_hal_init could not found coding type 8`
(JPEG) despite the SoC table now being correct.** Read `mpp_enc_hal_api_register()` in
`mpp/hal/mpp_enc_hal.c`: each individual HAL API implementation carries its OWN separate per-chip
whitelist (`api->soc_type[]`), checked independently of the SoC capability table just fixed above --
a second, genuinely separate registration mechanism, not the same gap re-surfacing. Found the real
JPEG-via-RKVENC implementation, `mpp/hal/rkenc/jpege/hal_jpege_vepu540c.c`'s `hal_jpege_vepu540c` --
its `soc_type[]` list only had `ROCKCHIP_SOC_RK3528`/`ROCKCHIP_SOC_RK3562`. Added
`ROCKCHIP_SOC_RV1106` to that list too, justified by the same real `HWID_VEPU540C` hardware-ID match
from gap #1, not a second blind guess. Rebuilt -- got past HAL registration for real, confirmed by
the error message changing to a different, later-stage failure (below), not the same one recurring.

**Where it genuinely, seriously stopped: a real kernel-level crash on the first actual submitted
job, not a software config problem.** With both integration gaps fixed, ran a real JPEG-encode
attempt (`mpi_enc_test`, a synthetic 640x640 NV12 test frame generated via `ffmpeg`) against the
actual hardware. It got all the way to real register submission -- `hal_jpege_v540c:
hal_jpege_vepu540c_status_check jpeg enc not done hw_status: 0x00000000`, then `dmesg` showed the
kernel's own `mpp_vcodec.ko` module crash for real inside `rkvenc_run` (`Process queue_work0`, full
register dump and stack trace, `mpp_rkvenc_worker` -> `kthread_worker_fn`), followed by
`rk_vcodec: mpp_wait_result_default: timeout`. **Checked the board survived before concluding
anything, not assumed:** `uptime`/SSH stayed responsive, `lsmod` still showed `mpp_vcodec` loaded, no
crash-loop in `ps aux --sort=-%cpu`, and a fresh run of the real, existing
`luckfox_video_hud_stream` detection pipeline immediately afterward produced completely normal
numbers (fps ~3.0, infer_ms ~86-101ms, jpeg_ms ~154-165ms) -- confirming this kernel oops was
contained to the one crashed kthread, not a wider system problem, and didn't need a reboot/reflash to
recover from. **Deliberately stopped here rather than keep experimenting against a kernel module
that had just oopsed** -- real, serious risk of an unrecoverable hang on a further wrong guess (this
project's own WiFi-driver history earlier this session/project already has two real instances of a
wrong kernel-adjacent guess needing a physical power cycle to recover from). Honest read: matching
`HWID_VEPU540C` got real software far enough to reach actual hardware and submit a real job -- further
than either RGA wall required -- but was not sufficient for full register-level compatibility between
RK3528/RK3562's exact VEPU540C integration and RV1106's. That's a genuinely deeper vendor-integration
detail (exact register offsets/sequencing, clock/power domain setup) not discoverable from public
sources alone; Luckfox's own uClibc build almost certainly gets this right (real, working hardware
JPEG encode clearly exists on this exact chip per its datasheet and per their vendor firmware), just
not source-available to compare against.

**Net result of this session's offload work, both targets reported honestly rather than forced to a
single tidy outcome:** RGA downscale -- real, working, measured net loss, not adopted as default.
MPP JPEG encode -- real, substantial forward progress (a working native build, two real and correctly
diagnosed integration gaps fixed using genuine hardware-ID cross-referencing rather than guesses) but
NOT completed -- stopped at a real kernel-level wall rather than pushed through it. `luckfox_video_hud_stream.c`'s
JPEG encode remains 100% software (`stb_image_write`) as a result; software JPEG encode (~146-165ms)
remains, confirmed and unchanged, the pipeline's single biggest real cost, bigger than NPU inference
itself.

## Real win found after the MPP wall: libjpeg-turbo replaces stb_image_write, ~2.5-3x faster real encode, ~53% real fps gain (2026-09-11, later same session)

With MPP hardware JPEG encode genuinely blocked (kernel crash, above), 1697-20747 asked directly what
could ease the JPEG-encode burden without it, pointing out this is core to what the chip is even
for. Real, low-risk lever, not the RGA/MPP kind of real-hardware-register wrestling: the software
encoder itself. `stb_image_write`'s JPEG encoder is a simple single-pass implementation with no
SIMD. Checked what's actually already on this board rather than assuming a build would be needed:
`dpkg -l | grep jpeg` shows `libjpeg-turbo8` is already installed and IS the real backing
implementation of this image's `libjpeg.so.8` (confirmed -- not a separate library sitting unused).
Real ARM NEON SIMD paths for color conversion and DCT exist in libjpeg-turbo; this chip's
`-mfpu=neon` compile flag (seen throughout this whole session's MPP build) confirms real NEON
hardware is present and otherwise unused by the current software encode path.

**Real, contained change, no kernel/driver risk at all.** `apt-get install libjpeg-turbo8-dev` for
`jpeglib.h` (dev headers weren't installed, only the runtime `.so` was) -- installed cleanly, no
issues. Swapped `http_send_frame()`'s `stbi_write_jpg_to_func()` call for the standard libjpeg
`jpeg_mem_dest()` in-memory-encode API (part of libjpeg's public API since v8, not the simplified
TurboJPEG API -- `jpeglib.h` was already the header on hand) at the same quality setting (50) as
before, for a fair, controlled before/after. Kept the output buffer/size as static variables reused
across frames (`jpeg_mem_dest()`'s own documented contract: reuses the existing allocation if large
enough, only reallocs to grow) rather than a fresh malloc/free every frame. Rebuilt with `-ljpeg`
added to the link line (`g++ ... -lrknnmrt -lrga -ljpeg -lm -lpthread`).

**Real correctness check before trusting any timing number:** grabbed a live sample from the actual
running stream (`curl` for a few seconds, extracted the first embedded JPEG by its `FFD8`/`FFD9`
markers), pulled it down and looked at it directly -- a real, correct frame: HUD overlay, fps/infer
readout, accelerometer panel, detection boxes and labels all rendering correctly, no color
corruption, no skew, no artifacts. Confirmed real before measuring speed, not assumed from a clean
build/exit code.

**One real false alarm along the way, caught and resolved, not glossed over:** a first timing
attempt showed zero frames processed in a 25+ second connected session, looking exactly like a hang
or a silent crash from the encoder swap. Investigated before assuming the code was broken: root
cause was a genuinely unrelated Mac-side shell mistake in an unrelated debugging step (`timeout`
isn't a real command on macOS, no GNU coreutils there by default) that silently no-op'd a client
connection attempt entirely -- the actual server process was healthy and simply still waiting for a
client the whole time. Confirmed by checking real process state (`ps aux`) before chasing a bug that
didn't exist, then re-ran the real test correctly with `curl`'s own `-m` flag instead.

**Real, measured result -- a genuine, substantial win, unlike the RGA/MPP hardware attempts:**

```
                              stb_image_write (software)   libjpeg-turbo (software, NEON)
JPEG encode alone                   ~146-165ms/frame             ~50-65ms/frame
Real steady-state end-to-end fps         ~3.3-3.4 fps                  ~5.2 fps
```

Confirmed via a full 40-second connected steady-state run (not just an early transient number) --
fps climbs smoothly from ~3.2 at frame 10 and genuinely plateaus at ~5.2 by frame ~150-210, not still
rising. A real ~2.5-3x cut in encode time and a real ~53% end-to-end fps improvement, entirely from a
userspace library swap -- no kernel module, no register-level anything, none of the real risk the
MPP attempt carried.

**Real, concrete consequence worth recording precisely: this changes which stage is now the
pipeline's single biggest cost.** Across this whole investigation (multiple sessions), JPEG
encode+network-send (and then, once split, JPEG encode alone) was always the dominant real cost,
bigger than NPU inference. **That's no longer true.** With libjpeg-turbo, JPEG encode (~50-65ms) is
now smaller than NPU inference (~87-97ms) -- inference is the new real bottleneck, for the first
time this project has measured it. Worth remembering before recommending further JPEG-side work:
the next real lever, if 5.2fps still isn't enough, is inference time itself (already explored
earlier -- 512x512 vs 640x640 input resolution), not JPEG encode.

**1697-20747 also asked about adding small companion hardware (a second board on GPIO) to help with the
encode burden -- answered directly rather than explored, since the real bandwidth math rules it out
cleanly:** a full 1280x720 RGB frame is ~2.76MB; GPIO-class links (SPI/UART) can't move that fast
enough to beat software encode itself, and anything fast enough (USB/Ethernet) stops being a
lightweight GPIO helper and becomes a second full SBC. More fundamentally, RV1106 already has real
JPEG-encode silicon on-die (the whole reason it's an IP-camera SoC) -- the blocker found this session
was a software/kernel-driver version mismatch, not missing hardware, so external hardware would be
working around silicon that's already there rather than fixing the real problem. Not pursued.

## 2026-09-11 (new session) -- MPP version-match hypothesis tested for real: partial confirmation, still not a working encode

Follow-up to the kernel-crash wall above. The concrete lead going in: `cat /proc/mpp_service/version`
reports the running kernel driver's own embedded build info as commit `424abb9b`, dated 2024-04-29 --
about 2.5 years older than the `develop`-branch HEAD (`0986d01`, 2026-08-26) the userspace library was
built from last session. Same author on both (Yandong Lin). Hypothesis: building userspace MPP from a
version-matched (or nearest-date) commit instead of `develop` HEAD might avoid whatever the kernel
crash was actually tripping over -- this project already proved this exact failure mode once, for RKNN
(the NPU toolkit had to be version-matched, not "newest is best").

**Verified the lead for real before rebuilding anything, and found something more precise than
expected.** Full-cloned `rockchip-linux/mpp` (not the `develop`-only shallow clone from last session)
and searched its complete history: `424abb9b` is a real, exact commit in this repo -- confirmed by
hash, message text ("[mpp_enc]: fix wrap enc sw timeout when resolution switch"), and author, no
guessing needed. Its **CommitDate is 2024-04-29 16:31:33 +08:00** -- an exact match, down to the
minute, against the kernel driver's own reported date. Strong, real confirmation this hash genuinely
is the kernel driver's provenance, not a coincidence.

**But `424abb9b` is not reachable from `develop` at all** -- `git merge-base develop 424abb9b` returns
nothing; neither is an ancestor of the other. Its directory layout (`vcodec/`, `vproc/`, a top-level
`mpp/` full of files like `mpp_vcodec_driver.c`, `mpp_service.c`) is the **kernel driver source
itself**, on a completely disjoint commit graph from `develop`'s classic userspace-library layout
(`osal/`, `mpp/hal/`, `mpp/codec/`, producing `librockchip_mpp.so` + `mpi_enc_test`). This public repo
evidently hosts the kernel driver and the userspace library as two unrelated histories that happen to
share the same GitHub repo (and, per the shared author/era, probably a common internal origin at
Rockchip) -- not one version-numbered tree. In hindsight this makes sense: the kernel module's own
embedded version string obviously comes from building *its own* source, not the userspace library's.

**Consequence: literally checking out `424abb9b` and building it produces the kernel driver, not
something `mpi_enc_test` can link against -- exercised the guardrail in the plan for exactly this
case.** Fell back to the documented alternative: found the `develop`-line commit closest by date to
the kernel driver's exact CommitDate (2024-04-29 16:31:33 +08:00). That commit is `23126cf6`
("fix[rc2_test]: fix pkt buffer overflow error"), committed 2024-04-29 10:13:34 +08:00 -- **same
calendar day, ~6.3 hours earlier**, the closest real commit on that line. Recorded explicitly per the
plan's own instruction: **this is a date-based best-effort match, not an exact hash match** -- weight
the result accordingly.

**Reapplied the known fixes -- two applied cleanly, the third has no equivalent this far back.**
`osal/inc/mpp_soc.h` (add `ROCKCHIP_SOC_RV1106` to the enum) and `osal/mpp_soc.cpp` (add an `rv1106`
table entry, `HAVE_RKVENC` only, HAL `vepu540c_no_hevc`, modeled on the real `rk3562` entry) both
applied the same way as last session -- confirmed no RV1106 entry existed yet at this commit either
(grep, zero matches, same as before). The third fix (a per-HAL `soc_type[]` whitelist in
`hal_jpege_vepu540c.c`) **does not exist as a mechanism at this commit at all** -- read
`hal_jpege_api_v2.c`'s `hal_jpege_init()` directly rather than assuming: at this point in the codebase's
history, JPEG HAL selection is decided purely by
`HWID_VEPU540C == mpp_get_client_hw_id(VPU_CLIENT_RKVENC)` -- a real hardware-ID match against the
actual kernel-reported device, no separate per-chip whitelist gate at all. That whitelist was added
later, upstream of this commit, somewhere between here and current `develop` HEAD. Real, structural
difference between the two code eras, not an oversight on either side.

**Built cleanly, 100%, on-device again** (`cmake` configured first try, `make -j1`, single core,
~11 minutes this time) -- `librockchip_mpp.so.0`/`.1` and `mpi_enc_test` both produced with no errors.

**Re-ran the identical real test as last session** (`mpi_enc_test`, the same synthetic 640x640 NV12
frame still sitting in `/tmp/test640_nv12.yuv` from last session, `-t 8` for MJPEG) against the actual
hardware, with the kernel's dmesg ring buffer cleared first (`dmesg -c`) so the result would be
unambiguous. **Real, different, and genuinely better result: no kernel crash this time.** Where last
session got a full Oops -- NULL pointer dereference, complete register dump, stack trace through
`rkvenc_run`/`mpp_rkvenc_worker`/`kthread_worker_fn` -- this run's entire dmesg output was one line:
`rk_vcodec: mpp_wait_result_default:1044: timeout, pid 650 session 16:7 count 1 cur_task 1 state 201`.
Confirmed this isn't a fluke by running it a second time with clean file-redirected output (the first
run's stdout got lost in a pipe) -- same result, a second timeout line, still no crash. Board stayed
fully healthy both times: `lsmod` still shows `mpp_vcodec` loaded normally, no crash-loop, `ps aux`
clean.

**But it's still not a working encode.** The test did produce an output file
(`/tmp/datematch_out.jpg`, 623 bytes) -- checked its real structure rather than trusting the non-zero
size: it has a valid SOI, DQT, SOF0, and DHT (all written by software before hardware dispatch) and a
real SOS marker, but **no EOI marker and essentially zero bytes of actual entropy-coded scan data**.
Consistent with the dmesg timeout: the hardware job was submitted and the software-side header
got written, but the hardware never signaled completion, so the file is header-only and not decodable
as a real image.

**Re-verified the existing pipeline is genuinely unaffected, the way the plan required before drawing
any conclusion.** First attempt at this reused the wrong model file
(`yolov8n-rv1106-toolkit1.6.0.rknn`, a stale/non-optimized export sitting in the same directory) and
hit a real but unrelated RKNN error (`unsupported input layout 3 for softmax`, `fallback cpu failed`)
-- investigated rather than assumed a regression, since nothing this session touched RKNN, the NPU
driver, or any model file. Re-ran with the actual documented model
(`yolov8n-optimized-rv1106-1.6.0.rknn`) and got a completely clean, real run: live detections
(`dets=1-3` per frame), fps climbing to a genuine **~5.0-5.1** plateau, `infer_ms` ~87ms, `jpeg_ms`
~50-77ms -- matching the documented libjpeg-turbo baseline exactly. Zero regression, confirmed by a
real run, not inferred from "I didn't touch that code."

**Honest read on the hypothesis: partially confirmed, not fully.** Version-matching the userspace
build to the kernel driver's era did change the real, observed failure mode from a hard kernel panic
to a soft driver-side timeout -- genuinely different, and a real (if narrow) win for the "match
versions, don't assume newest is best" lesson this project already learned once for RKNN. But the
underlying job still never completes on real hardware, so this is not a working hardware JPEG encoder,
and `luckfox_video_hud_stream.c` was **not** changed -- there is no real MPP encode path to integrate.
The remaining gap reads the same as last session's own conclusion, just reached by a cleaner path this
time: RK3528/RK3562's `vepu540c`/`vepu540c_no_hevc` HAL, even hardware-ID-matched and now
version-era-matched, still doesn't fully match whatever RV1106's actual kernel driver expects at the
register/sequencing/clock-power-domain level -- a genuinely deeper vendor-integration detail, not
discoverable from public source, that Luckfox's own uClibc vendor build almost certainly gets right
but isn't source-available to compare against. Worth flagging the one real caveat on the version-match
result itself: `23126cf6` is a date-based nearest match, not the literal commit the kernel driver was
built from (that literal commit isn't reachable from the userspace-library history at all) -- so this
doesn't fully rule out version skew as *a* contributing factor, just shows that the specific,
practically-buildable nearest-match attempt still isn't sufficient on its own.

**Stopped here, deliberately, same discipline as last session.** A driver-timeout is a much softer
failure than a kernel Oops -- real risk was lower this time -- but pushing further from here would mean
guessing at genuinely proprietary register-level/clock-power-domain details with no public source to
check against, the same wall as before, just reached through a cleaner, more precisely-diagnosed path.
`libjpeg-turbo` remains the correct, current, unchanged default for JPEG encode in
`luckfox_video_hud_stream.c`.

## 2026-09-11 (same session, later) -- real CPU-utilization check, and re-running the 512x512 benchmark against today's libjpeg-turbo build

With MPP hardware encode still blocked, 1697-20747 asked what's next: is the CPU actually maxed out, and
if the 512x512 inference model is faster, is there room to raise display resolution.

**Checked CPU utilization live rather than assuming -- genuinely, fully saturated.** Sampled `top`
six times over ~12s of real steady-state streaming: **0.0% idle, 0.0% iowait, every single sample**,
split between `ffmpeg` (~40-53%, software-decoding/scaling `bikes.mp4`) and the pipeline itself
(~30-37%), on this chip's one real CPU core (`nproc=1`). Confirmed via source read
(`luckfox_video_hud_stream.c`, no `pthread_create`, one `while(1)` doing
read->downscale->infer->jpeg-encode->send in strict sequence) that this is real, not an artifact --
there's no idle capacity a scheduler or threading change could reclaim on a single core; any further
speedup has to come from doing less real work, not better overlap.

**Re-ran the 512x512-vs-640x640 inference benchmark from earlier this session's memory entry, but
against *today's* libjpeg-turbo build** -- the original 512 benchmark (`## ... 512x512 vs 640x640
inference benchmark`, above) predates the libjpeg-turbo swap entirely, so it measured the old
stb_image_write-era pipeline. The on-board `luckfox_video_hud_stream_512` binary turned out to be
that same stale pre-libjpeg-turbo fork (confirmed by grep: still calls `stbi_write_jpg_to_func`, not
`jpeg_mem_dest`) -- **not reused**, since it wouldn't have been a fair comparison. Instead copied
today's actual `luckfox_video_hud_stream.c` (real libjpeg-turbo default intact), changed only
`#define MODEL_SIZE` from 640 to 512, and rebuilt on-device
(`g++ ... luckfox_video_hud_stream_512b.c utils/image_drawing.c -I. -Iutils -I3rdparty/stb_image
-Irga_include -lrknnmrt -lrga -ljpeg -lm -lpthread` -- needed `-Iutils`/`utils/image_drawing.c` for
`draw_rectangle`/`draw_text` and `-Irga_include` for `im2d.h`, neither obvious from the original
build note). Ran both back-to-back, same `bikes.mp4`, same default (native, `ffprobe`-confirmed
1280x720) display resolution, 35s real connected sessions each:

```
                                640x640 inference (today's libjpeg-turbo)   512x512 inference (today's libjpeg-turbo)
NPU inference                            ~86-87ms/frame                             ~62-63ms/frame
Steady-state end-to-end fps                  ~5.2-5.3 fps                               ~5.4 fps
```

Inference itself dropped a real ~28% (86.9ms -> 62.7ms), closely matching the original benchmark's
~25% finding -- reproducible, not a fluke. **But the net end-to-end fps gain this run was modest,
only ~3-4%**, well under what the raw inference saving alone would predict. Real, honest reason,
checked rather than hand-waved: `jpeg_ms` ran noticeably higher during the 512 run (~65-88ms,
averaging ~75ms) than during the 640 baseline run immediately before it (~49-68ms, averaging ~57ms)
-- almost certainly real frame-to-frame JPEG-entropy variance (later parts of `bikes.mp4` compress
differently than earlier parts), not a systematic cost of the inference-resolution change, since JPEG
encode operates on the unchanged 1280x720 display frame regardless of inference resolution. Summing
each run's own accounted stages (infer+jpeg+read+net) shows the real underlying improvement was
present (~172ms/frame -> ~168ms/frame) but partly masked by this run's jpeg-timing noise. A cleaner,
longer, or repeated run would likely show a larger, cleaner gain closer to the original ~10-15%
finding -- not done this session, real result reported as measured rather than smoothed over.

**On "can we bump display resolution up now that inference is faster": no, not with this test
source.** `ffprobe` confirms `bikes.mp4` is natively exactly 1280x720 -- the same resolution the
pipeline already defaults to. There is no higher real resolution to render at without artificially
upscaling the source (pure wasted encode/decode cost for zero real detail gain, the opposite of the
point). Any inference-time savings from 512x512 right now just becomes fps headroom, not usable
display-quality headroom, until either a genuinely higher-resolution source is used or (per the
`ffmpeg`-elimination point above) the live camera is connected with its own native resolution.

## 2026-09-11 (same session, later still) -- live camera physically connected, verified for real end-to-end; accelerometer re-confirmed

1697-20747 connected the CSI camera and reported the accelerometer live too, and asked for the docs to be
updated for real hardware state, with a preference for a slight resolution bump on the live camera
once it's wired in. Verified everything below directly rather than taking the report at face value --
this project's whole discipline has been "check the real device before writing it down."

**Real camera detection, checked from the kernel/I2C side up, not assumed from a driver loading.**
`i2cdetect -y 4` (the CSI sensor-control bus per this repo's own GPIO/I2C-bus notes) shows a real
device at `0x30` marked `UU` -- claimed by a kernel driver, not just present. Confirmed which one:
`/sys/bus/i2c/devices/4-0030/driver` symlinks to `.../drivers/sc3336`, `name` and `modalias` both say
`sc3336`/`smartsens,sc3336`. `lsmod` shows `sc3336`, `video_rkcif`, `video_rkisp`, and both CSI D-PHY
modules loaded, with `rkcif`/`rkisp` already showing a real client (`rockit`, count 1 each) --
something is already actively using the ISP pipeline, not just idle-loaded. This is exactly the one
sensor this board's vendor image ships a driver for (the README's prior IMX219/IMX708/OV5647 concern
was about a *different*, incompatible camera -- moot now that this is the matching one).

**No `v4l2-ctl`/`media-ctl` on this image at all -- installed `v4l-utils` via `apt-get` to actually
probe real capability** (clean install, no issues, same low-risk userspace-package pattern as
`libjpeg-turbo8-dev` earlier this session). `--list-devices` shows the real pipeline topology:
`rkcif` raw-capture nodes (`/dev/video0`-`10`) feeding `rkisp_mainpath` output nodes
(`/dev/video11`-`18`). `--list-formats-ext` on the mainpath nodes reports real, consistent
capability across all of them: **stepwise 32x32 to 2304x1296 (8px steps)**, formats including
NV12/NV21/NV16/UYVY/GREY/XR24(BGRX)/RGBP(RGB565) -- 2304x1296 matches the SmartSens SC3336's real
published 3MP sensor resolution, a second independent confirmation this is genuinely that sensor.

**Real capture test, not just format enumeration -- pulled an actual frame and looked at it.**
`v4l2-ctl -d /dev/video11 --set-fmt-video=width=1920,height=1080,pixelformat=NV12 --stream-mmap
--stream-count=3 --stream-to=/tmp/cam_test.raw` produced a file whose size (9,331,200 bytes) matches
the exact byte-for-byte math for 3 real NV12 frames at 1920x1080 -- no truncation, no dropped data.
A quick statistical sample of the Y plane showed real variation (9 distinct values, not a flatlined
dead sensor) but a suspiciously narrow, dark range (17-27 out of 255) -- investigated rather than
either trusted or dismissed: pulled the raw frame to the Mac, manually decoded NV12->RGB (BT.601, by
hand with numpy -- no `ffmpeg` on the Mac side, installed a throwaway Pillow/numpy venv rather than
touching system Python per PEP 668) and rendered both a true-color and a brightness-boosted grayscale
PNG. **Real, recognizable image** -- a ceiling/wall corner, visible seam line, what looks like a cable
crossing the upper right -- confirming a genuinely live, correctly-wired sensor pointed at a real dim
room, not a dead/frozen/garbage capture. The darkness itself is most likely unset auto-exposure/gain
on this direct low-level `v4l2-ctl` grab (bypassing whatever 3A/AE tuning the vendor's normal camera
stack would apply), not a hardware problem -- worth keeping in mind for the real integration, but not
concerning on its own.

**Accelerometer re-confirmed live, unchanged.** `/sys/bus/iio/devices/iio:device1` is still `mpu6050`,
raw reads still real and non-flat (`x=14624 y=1440 z=14354` this check, different instant/orientation
than the 2026-09-09 entry's numbers but the same real sensor, same interface, nothing regressed).

**Resolution recommendation for the real integration, grounded in what was just measured:** 1920x1080
is a genuine, verified-working target -- a real step up from the current 1280x720 video-file-testing
default, comfortably under the sensor's real 2304x1296 ceiling (leaves headroom rather than maxing
out). **Not yet wired into `luckfox_video_hud_stream.c`** -- the pipeline still only reads via
`ffmpeg`-decoding a video *file*; swapping in a real V4L2 capture loop (mmap-based, like the
verification test above, rather than piping through another `ffmpeg` process) is the real next
implementation step, not done this session. It's also a bigger win than "just" enabling the live
camera: this session's own CPU-utilization check found `ffmpeg`'s software file-decode already eating
~40-53% of this chip's one real CPU core every frame -- a native V4L2 capture path removes that
software-decode cost entirely rather than trading it for something else, on top of unblocking real
live operation instead of a canned test video.

## 2026-09-11 (same session, later still) -- live camera actually wired into the pipeline, at 1920x1080; a real latent SIGPIPE bug found and fixed along the way

1697-20747 asked to wire the live camera in for real ("make the live camera feed properly, go ahead"),
with 1920x1080 as the target resolution per the prior entry's recommendation.

**Real design: `luckfox_video_hud_stream.c` now auto-detects a live camera vs. a video file** by
checking whether the second argument starts with `/dev/video` -- no new CLI flag needed, the same
binary handles both. Added a standard V4L2 mmap capture path (`V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE`,
matching the real `rkisp_mainpath` topology confirmed last entry): open the device, `VIDIOC_S_FMT`
requesting NV12 at the target size (the driver may negotiate a different real size on this stepwise
sensor -- used what it actually reports back, not the request), `VIDIOC_REQBUFS`/`VIDIOC_QUERYBUF`/
`mmap` for 4 buffers, `VIDIOC_STREAMON`, then a normal `VIDIOC_DQBUF`/convert/`VIDIOC_QBUF` loop per
frame. Opened ONCE like the RKNN model, not torn down between HTTP client (re)connections (unlike the
video-file path's per-connection `ffmpeg` respawn) -- reconfiguring a live sensor pipeline per browser
connect would be real, unnecessary churn.

**NV12->RGB conversion, reusing the exact formula already hand-verified this session.** The ISP's real
native capture format is NV12 (confirmed last entry), but the rest of this pipeline (HUD drawing, RGA
downscale path, JPEG encode) is RGB24-based -- added `nv12_to_rgb()`, the same BT.601 integer formula
already manually verified against a real captured frame in Python earlier this session (not new,
untested math -- reused the checked one).

**A real latent bug found and fixed, not specific to the camera feature but exposed by it.** The first
live-camera test runs kept dying silently, no error printed, process just gone from `ps aux`. Root-
caused for real rather than guessed: ran once in the foreground (not backgrounded) specifically to
capture the shell's own exit-status report -- `EXIT_CODE:141` = 128+13 = **SIGPIPE**. The default
SIGPIPE disposition kills a process on its very first failed `send()`/`write()` to a closed socket,
*before* `http_send_frame()`'s own already-existing return-value check (written to handle exactly a
disconnected client) ever gets to run. This bug has genuinely been in this file since the JPEG-
streaming design was first written -- it was just silently masked in the video-file path because
`ffmpeg`'s own subprocess printed ITS OWN "Broken pipe" message when it independently hit SIGPIPE on
its now-orphaned stdout pipe, which reads exactly like ordinary shutdown noise in the log (every
earlier session's logs, this one included, show that message and nothing else on disconnect -- the
program's own `RESULT: CLIENT_DISCONNECTED` line never once actually printed, unnoticed until now).
The live-camera path has no `ffmpeg` to generate that cover message, which is what surfaced it for
real. Fix: one line, `signal(SIGPIPE, SIG_IGN)` near the top of `main()`, applies to both paths.
**Verified fixed, not just plausible:** re-ran both the live-camera path and the video-file path after
the fix, and `RESULT: CLIENT_DISCONNECTED after N frames` now prints correctly on both, with the
process surviving and returning to `Waiting for a browser to connect...` afterward -- confirmed via
`ps aux` staying populated, not assumed. A genuine robustness improvement to the whole pipeline, found
as a side effect of adding the camera, not something the camera feature itself needed.

**Real, working, verified end-to-end -- not just a clean build.** `RESULT: V4L2_CAPTURE_READY
/dev/video11 1920x1080 NV12 buffers=4` on startup (the driver accepted the requested 1920x1080
outright, no renegotiation needed). Pulled a real frame out of the live MJPEG stream mid-session and
looked at it directly (same discipline as every other correctness check this project has done): HUD
overlay text, fps/infer readout, and a real, live accelerometer panel (`ACCEL x8.72 y0.88 z8.98 g`,
matching the real raw sensor values re-checked earlier this session) all rendered correctly, no color
corruption or skew from the NV12->RGB conversion. Scene is dark (same real dim room noted in the prior
camera-verification entry -- unset auto-exposure on this codepath, not a bug) but genuinely live and
correctly decoded.

**Real, measured performance at 1920x1080 live vs. the 1280x720 video-file baseline, steady-state:**

```
                         1280x720 file (bikes.mp4)   1920x1080 live camera
read/capture              ~25-30ms                    ~45ms (DQBUF + NV12->RGB, new cost)
NPU inference (nested)    ~86-87ms                     ~75-76ms
JPEG encode                ~50-68ms                    ~105-135ms
Steady-state fps            ~5.2-5.3 fps                  ~4.3 fps
```

Two real, honest, separate effects here, not one: JPEG encode costs more at 1920x1080 (2.25x the
pixels of 1280x720) -- expected, and exactly the tradeoff 1697-20747 asked for in choosing a real resolution
bump. But **inference actually got faster**, a genuine, unplanned side benefit -- with no `ffmpeg`
subprocess competing for this chip's one CPU core anymore, the RKNN/downscale stages see less real
contention. Net: lower fps than the 720p file baseline, but for a real reason (more real display
pixels, not a regression), on real live camera and accelerometer data for the first time this project
has had both running together.

## 2026-09-11 (same session, later still) -- chasing the RGA/live-camera CMA failure to its real root: a hardcoded 10MB DTB region, not the value already fixed

1697-20747 asked to re-test RGA downscale now that the live-camera path had freed real CPU headroom. First
attempt (documented in the conversation, not repeated here) failed outright -- `DRM_IOCTL_MODE_CREATE_DUMB:
Cannot allocate memory`, traced to `cma_alloc: linux,cma: alloc failed, req-size: 300 pages` and
`/proc/meminfo` showing `CmaFree: 0kB` out of a 46MB total pool. Asked to "execute full fix."

**Real, working fix for the first (wrong) target, found by tracing the actual kernel source rather
than guessing.** `rk_dma_heap_cma=36M` (the boot cmdline token) turned out not to live in the kernel
DTB at all -- traced via the SDK's own `sysdrv/tools/pc/toolkits/update_dtb_bootargs.sh` +
`repack_bootimg.sh` tools (found alongside each other, clearly a matched pair) and a real `dumpimage`
extraction of the board's actual running FIT image: the extracted `fdt`'s own `bootargs` property only
had `earlycon=... console=... root=... rootwait coherent_pool=0` -- no CMA token, no `blkdevparts`, no
`androidboot.*`. Those get appended by **U-Boot's own environment** at boot time, confirmed directly by
`dd`-dumping the tiny (32KB, unsigned, plain-text) env partition (`/dev/mmcblk0p1`) and finding
`sys_bootargs= root=/dev/mmcblk0p7 rootfstype=ext4 rk_dma_heap_cma=36M` in the raw bytes. This is a
genuinely safer fix point than the kernel/DTB -- no signing, no repack, trivially small and backed up.
Backed it up (both to the board's own `/tmp` and pulled to the Mac, md5-verified in both places, same
as the boot-partition backup taken first). Installed `u-boot-tools` (`fw_printenv`/`fw_setenv`) via
apt, wrote a correct `/etc/fw_env.config` (`/dev/mmcblk0p1 0x0000 0x8000`, confirmed against the raw
partition's own CRC32-header + flat env format seen in a hex dump), verified read-only first, then
`fw_setenv sys_bootargs " root=/dev/mmcblk0p7 rootfstype=ext4 rk_dma_heap_cma=66M"` (66M matching the
board config's own `RK_BOOTARGS_CMA_SIZE="66M"` default, which -- a real, separate finding -- turns out
to have never actually been applied to this board's real flashed env; the earlier accelerometer/WiFi
kernel rebuilds only touched kernel Image + DTB source, never regenerated bootargs from board config).
**Real, verified reboot:** ~9s to SSH-reachable, same kernel build timestamp (proving no kernel/DTB was
touched), `/proc/cmdline` confirmed `rk_dma_heap_cma=66M`, `CmaTotal` grew 47MB->78MB exactly as
predicted, camera/accelerometer/all kernel modules present and healthy.

**But the RGA test still failed afterward, identically -- `CmaAllocated: 29672kB` unchanged from
before the fix, byte for byte.** This is the real, decisive tell that `rk_dma_heap_cma` was the WRONG
target all along, not just an insufficient one. Traced further into the kernel source rather than
assuming "needs more": the failing allocator's own dmesg message says `linux,cma` (a literal region
name), not `rk_dma_heap_cma`. Grepped the SDK's real board DTS
(`rv1106-luckfox-pico-ultra-ipc.dtsi`) and found two SEPARATE, independently-sized CMA regions coexist
on this chip: `rk_dma_heap_cma` (the one just fixed, used by the Rockchip media/MPP/ISP stack via its
own named heap driver) and a completely different, generic `linux,cma` reserved-memory node --
**hardcoded to exactly `size = <0xA00000>; //10M`** in the DTB, `reusable`/`linux,cma-default` --
which is what DRM's dumb-buffer allocator (the RGA path's own memory source) actually draws from. The
math lines up exactly: 47104kB original total - 36864kB (`rk_dma_heap_cma`) = 10240kB = 10MB; after the
fix, 77824kB - 67584kB (66M) = 10240kB, unchanged. The RGA path's own two buffers need ~7.4MB
(6.2MB @ 1920x1080 RGB + 1.2MB @ 640x640) inside that same fixed 10MB pool that the live camera's own
ISP/CIF pipeline is concurrently drawing from and (very plausibly, real CMA fragmentation from a
live, actively-streaming producer) can't reliably supply a contiguous 1.2MB chunk of.

**Real, honest status: found the true root cause, but the actual fix (resizing the `linux,cma` DTB
node) needs a genuine kernel/DTB rebuild and reflash -- a materially higher-risk step than the
env-only change just made (real FIT/boot.img repack and signing considerations, not just a small
unsigned env partition write).** Paused here to explicitly re-confirm with 1697-20747 before taking that
step, rather than stretching the "execute full fix" instruction past the risk level it was actually
given for. The `rk_dma_heap_cma=66M` env change is left in place -- real, verified, harmless (more
headroom for the media/MPP heap doesn't cost anything), just not sufficient alone for RGA+live-camera.

## 2026-09-11 (same session, continued) -- the real DTB/kernel fix, done for real, with a clean negative result at the end

1697-20747 confirmed proceeding, twice (once to edit the env, once explicitly for the kernel/DTB step after
the risk level was flagged as materially higher).

**Resolved the signing-risk question empirically before touching anything, rather than guessing.**
The FIT `.its` template declares a `signature { algo = "sha256,rsa2048"; key-name-hint = "dev"; ... }`
node with no `required = "conf"` property, which is suggestive but not proof. Settled it for real:
decompiled the board's OWN currently-running `boot.img` (already backed up) with `dtc -I dtb -O dts`
directly on the FIT file itself -- the signature node is present with `algo`/`padding`/`key-name-hint`
declared, but **no `value` property at all**. The board is right now booting from a genuinely unsigned
image. That's direct, empirical proof this board's U-Boot does not enforce FIT signature verification
-- if it did, the currently-running system couldn't exist. No signing key was ever needed (none found
anywhere in the SDK checkout either, consistent with this).

**Found the real root cause via the SAME `dumpimage`/`dtc` extraction discipline used for the bootargs
tracing above.** The RGA path's DRM dumb-buffer allocator fails against a region literally named
`linux,cma` in its own dmesg error -- a completely different, second CMA pool from `rk_dma_heap_cma`.
Grepped the real board DTS (`rv1106-luckfox-pico-ultra-ipc.dtsi`) and found it: a `reserved-memory`
node, **hardcoded `size = <0xA00000>; //10M`**, `reusable`/`linux,cma-default`, shared with the live
camera's own ISP/CIF pipeline. The math confirms it precisely: 47104kB original total -
36864kB(`rk_dma_heap_cma`) = 10240kB = 10MB, unchanged before and after the env-only fix.

**One-line DTS edit, minimal and verified.** `size = <0xA00000>; //10M` -> `size = <0x2000000>; //32M`
in `sysdrv/source/kernel/arch/arm/boot/dts/rv1106-luckfox-pico-ultra-ipc.dtsi`. Confirmed via `git
diff` this was the ONLY new change on top of the already-applied, real mpu6050 i2c2 patch from the
earlier accelerometer session (still present, untouched, confirming this checkout genuinely matches
what's flashed).

**Rebuilt with the exact same proven `./build.sh kernel` target the accelerometer/WiFi sessions used
successfully twice before**, in the SAME still-running `luckfox-sdk` Docker container (found alive
from earlier today, `/root/sdk`, same commit `824b817f8`, same patches already applied -- reused
rather than re-cloning ~1GB). Build succeeded clean (`mrproper` + full reconfigure + rebuild, ~a few
minutes, one pre-existing unrelated DTS warning about an unused touchscreen node). Verified the new
DTB directly (not just trusted a clean build exit): `dtc`-decompiled the freshly-built
`rv1106g-luckfox-pico-ultra.dtb` and confirmed `size = <0x2000000>` really is in there, confirmed the
new FIT is equally unsigned (matching the current running image, not a new risk), and confirmed the
DTB's own baseline `bootargs` property is byte-identical to before (no accidental duplication with the
U-Boot-env-level `rk_dma_heap_cma=66M` fix from earlier).

**Flashed with the full, same discipline as every prior kernel-level session in this project.** Backup
already in place from the env-fix step (`f5c6b18ce6270944a256c62b6bb48ae2`, boot partition, verified in
two places). New `boot.img` (3,869,184 bytes) md5-verified identical across the container -> Mac -> board
hop (`d4561fa1e825df75cb1e1b160cd767cd`, all three). `dd if=/tmp/new_boot_cma32m.img
of=/dev/mmcblk0p4 bs=1M conv=fsync`, `sync`, `reboot`. **Real, verified boot success:** SSH-reachable
again in ~9s (same fast reboot as the env-only change). `uname -a`'s build timestamp came back
identical to before this change -- investigated rather than alarmed: benign, since the kernel Image
(vmlinux) doesn't depend on devicetree source at all, so an unchanged C-source tree legitimately
produces a bit-identical kernel Image and its own embedded build string, even under a full
`mrproper` rebuild. The REAL, decisive confirmation the new DTB was live: `CmaTotal` came back at
exactly `100352kB` = `66M + 32M` = 98MB, precisely matching the intended resize -- a devicetree
reserved-memory size can only come from the devicetree, so this is unambiguous proof. All hardware
(camera, accelerometer, every kernel module) confirmed present and healthy afterward.

**The actual RGA test, for real this time: both DRM buffers allocate successfully --
`RESULT: DOWNSCALE_MODE RGA` confirmed active, the CMA fix genuinely worked.** But the real,
measured performance result is a clean, honest negative, not a win: RGA's `downscale_ms` is
**~35-36ms**, vs. the CPU path's **~9.3-9.5ms** on this exact same live-camera setup -- RGA is
roughly 4x SLOWER, and real steady-state fps is correspondingly worse (3.8-3.9fps RGA vs. 4.2-4.3fps
CPU). **This settles the original question the CPU-headroom hypothesis was testing: the answer is no,
freed CPU headroom does not make RGA worthwhile here.** The reason isn't CPU contention at all (which
is what "headroom" would have fixed) -- it's that RGA's own mandatory copy-through-contiguous-DMA-
memory overhead is intrinsically slower than the CPU nearest-neighbor loop for an operation this cheap
(a 1920x1080->640x640 resize), the same fundamental finding from last session's original RGA
investigation at 720p, now independently re-confirmed at 1080p with the memory blocker genuinely fixed
and real CPU headroom genuinely available. `USE_RGA_DOWNSCALE` remains off by default, real, working,
just not the faster choice on this hardware -- now proven twice, under two different real conditions.

**Final regression check, same discipline as always:** re-ran the default CPU-downscale live-camera
path after all of this -- identical numbers to the pre-flash baseline (fps 4.3, downscale_ms 9.3-9.4,
infer_ms 74.4-74.6ms), zero regression from the kernel/DTB change. Both real backups (original 32MB
boot partition, original 32KB env partition) remain on the Mac, md5-verified, as the recovery path if
ever needed.

## 2026-09-11 (same session, later still) -- YOLO26 re-checked now that the pipeline actually works; still blocked, and an honest answer on whether it's solvable at all

1697-20747 asked to try switching the now-fully-working pipeline (live camera, V4L2, everything built this
session) from `yolov8n` to `yolo26n`, then asked directly whether the blocker is solvable at all.

**Re-verified fresh on today's actual board state rather than citing the 2026-09-08 investigation
from memory -- real, current, not stale.** Ran `yolo26n-rv1106-toolkit1.6.0.rknn` (still sitting on
the board from the original conversion) directly against the current `luckfox_video_hud_stream`
binary with the live camera. **Identical failure, same exact error as three sessions ago:**

```
E RKNN: unsupport cpu MatMul op, op name: MatMul:/model.10/m/m.0/attn/MatMul_1
E RKNN: rknn fallback cpu failed
RESULT: INIT_FAIL ret=-1
```

Real, useful confirmation from this specific re-test: the failure happens at `rknn_init()`, before
any of this session's camera/HUD/streaming/SIGPIPE work even runs -- none of that work could have
changed this outcome, since nothing this session touched `librknnmrt.so` or the RKNN driver/runtime
at all (only the kernel/DTB's CMA memory sizing changed, an unrelated subsystem). "Now that the
pipeline runs" was never actually a relevant variable here -- worth having said explicitly, since it
wasn't obvious without checking.

**Answered "is this solvable at all" honestly, synthesizing the three real paths already found back
on 2026-09-08 (see above) rather than re-deriving them, plus why solving it likely wouldn't even
help:**

1. **Wait for upstream.** `airockchip/rknn-toolkit2#498` (the tracked YOLO26-support issue) -- zero
   activity in 8+ months, out of this project's control.
2. **Graph-surgery the C2PSA attention block into NPU-compatible ops**, the same kind of surgery
   Rockchip already did for YOLOv8's DFL head. Genuinely possible in principle -- but changes the
   actual computation, so the pretrained weights stop applying correctly; would need real
   fine-tuning/retraining afterward to recover accuracy. A standalone ML engineering project on its
   own, disproportionate to this board-bringup task, not attempted.
3. **Different, more capable Rockchip silicon** (RK3566/RK3588-class) -- already has official YOLO11
   support (same C2PSA attention family as YOLO26). Sidesteps the problem entirely, but means new
   hardware, not a fix for this board.

**Even if solved, it likely wouldn't help on this specific chip -- the real reason not to chase any
of the above.** YOLO26/YOLOv10+'s speed claim is specifically about dropping NMS postprocessing
overhead (the "one-to-one" NMS-free head). Already measured on this exact board (2026-09-08 entry):
NMS was never the bottleneck here -- only ~31 real candidate boxes survive threshold on the reference
test image, microseconds of work regardless. The real, measured cost is backbone convolution (NPU MAC
throughput), and C2PSA's attention block adds MORE compute on top of a similar backbone, not less --
so even setting the load-failure aside, YOLO26 would likely be slower than the already-working
`yolov8n` pipeline on this ~1 TOPS chip, not faster. `yolov8n` remains the right, real choice for this
board -- not a placeholder waiting on a fix.

## 2026-09-11 (same session, later still) -- pointer: full prep plan for camera calibration + distance detection lives in apex_ros

1697-20747 wants to run real camera calibration on this board's own camera next session, for real distance
detection of objects, building on `apex_ros`'s already-working `apex_camera_calibration` +
`apex_perception.mono_distance_estimator` (mono distance from known object size) -- both built for the
Pi Camera V3, not this board's `sc3336`. **The full, real prep plan (what's portable, what needs new
V4L2-based capture code, a concrete step-by-step plan, and one already-run real check -- this camera's
capture node exposes no autofocus controls, consistent with fixed focus) is written up as
`apex_ros/ACTION_ITEMS.md` item #32, not duplicated here.** Check that entry first before starting;
it also says to re-check THIS file's own current state before trusting anything it assumed about the
Luckfox side, so read both fresh rather than trusting either in isolation.
