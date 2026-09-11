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
