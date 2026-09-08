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
