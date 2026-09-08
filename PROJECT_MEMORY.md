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
Rockchip's own CDN and re-ran the same toolkit-1.6.0 conversion against it -- **result pending,
not yet tested on real hardware as of this note** (see next session's update / check
`convert_v8n_opt_log.txt`-equivalent in this repo's history for the real outcome, don't assume).

**Note also, for completeness: RV1103/RV1106 is not in the "Current Support Platform" list even
for this officially-optimized yolov8 export** -- so even a clean result here would be an
unofficial-but-empirically-working configuration, not a documented-supported one.

## What this means for the original goal (YOLO26 offload, replacing slow Pi-5-CPU inference)

Not resolved as of this note. The honest picture: this specific board's specific real hardware
constraints (a genuinely weak, RV1103/1106-family "mini" NPU, an old vendor runtime with no newer
version to move to, and the zero-copy-only memory API) make it a harder target than the Hailo-8
path already working elsewhere in `apex_ros` for the same detection problem. If the Pi-5-CPU
framerate problem needs solving via NPU offload specifically, the honest options once this
investigation is done being run are: (1) accept yolov8n or an even older/simpler architecture here
if bug 2b turns out fixable, trading detection quality/recency for actually running; (2) a
different, more capable Rockchip chip (RK3566/RK3588-class) that officially supports YOLO11/likely
YOLO26 once official examples land, if a board swap is on the table; (3) stay on the already-working
Hailo-8 AI HAT+ path for this problem and treat the Luckfox as unsuitable for this specific model
family. None of these are decided -- recorded here as the real, current state of the tradeoff, not
a recommendation made without 1697-20747's input.
