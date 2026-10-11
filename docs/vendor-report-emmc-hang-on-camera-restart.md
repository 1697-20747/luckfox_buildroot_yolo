# eMMC stops completing writes ~5.5 s after a camera stream restart while rkaiq (3A) is active — Luckfox Pico Ultra (RV1106G3), Buildroot SDK

## Environment

| | |
|---|---|
| Board | Luckfox Pico Ultra (RV1106G3), 7.1 GB eMMC (`TWSC`, EXT_CSD rev 8, HS 52 MHz requested / 49.5 MHz actual, 8-bit) |
| SDK | `LuckfoxTECH/luckfox-pico` @ `824b817f8` ("Pullrequest 260319 (#355)"), `BoardConfig-EMMC-Buildroot-RV1106_Luckfox_Pico_Ultra-IPC.mk` |
| Kernel | 5.10.160 (SDK kernel; only additions: i2c2 MPU-6050 node, watchdog + ramoops + SysRq for diagnosis) |
| Camera | SC3336 (`m00_b_sc3336 4-0030`), IQ `sc3336_CMK-OT2119-PC1_30IRC-F16.json` |
| 3A | `librkaiq` from `media/isp/release_camera_engine_rkaiq_rv1106_arm-rockchip830-linux-uclibcgnueabihf` (`AIQ v5.0x5.0`), used both as `rkaiq_3A_server` and in-process via `rk_aiq_uapi2_sysctl_*` |
| eMMC health | EXT_CSD `PRE_EOL_INFO=0x01`, `DEVICE_LIFE_TIME_EST_TYP_A/B=0x01` (0–10 %) |

## Symptom

After a camera stream is **stopped and started again** (application restart), with rkaiq running,
the eMMC sometimes stops completing writes about **5.5–5.8 s after the stream is turned back on**.
The MMC core re-initialises the card (400 kHz → 49.5 MHz) without any error message being printed,
writes still fail, ext4 aborts the journal, and every task that touches storage blocks in
`io_schedule` forever. The system is otherwise alive (the watchdog keeps being fed), so it never
recovers on its own; only a reset does. The first stream start after boot has never failed.

Kernel log (captured via ramoops after a watchdog reset; the `yolo-server` lines are markers
written by the application to `/dev/kmsg`):

```
[   79.92] rockchip-mipi-csi2 mipi0-csi2: stream ON            <- 3rd stream start this boot
[   85.684067] mmc_host mmc0: Bus speed (slot 0) = 400000Hz (slot req 400000Hz, actual 400000HZ div = 0)
[   85.701141] mmc_host mmc0: Bus speed (slot 0) = 49500000Hz (slot req 52000000Hz, actual 49500000HZ div = 0)
[   88.721464] blk_update_request: I/O error, dev mmcblk0, sector 1124760 op 0x1:(WRITE) flags 0x800 phys_seg 2 prio class 0
[   88.721571] Aborting journal on device mmcblk0p6-8.
[  100.564052] mmc_host mmc0: Bus speed (slot 0) = 400000Hz ...
```

SysRq `w` at that point shows `jbd2/mmcblk0p7`, `syslogd` and every writer blocked in
`io_schedule` ← `bit_wait_io` ← `jbd2_journal_commit_transaction` / `do_get_write_access`.

The gap between `stream ON` and the card re-init was 5.76 s, 5.74 s, 5.81 s, 5.83 s and 5.52 s in
five separate captures. In one capture the application was idle (no NPU, no encoder activity)
during that window — only the camera stream and rkaiq were active.

## A/B results (restart stress: stop/start the camera application every 3–8 s)

| Configuration | Restarts | Failures |
|---|---|---|
| `rkaiq_3A_server` started once at boot, left running across stream restarts | ~25 + 30 | **5** + **3** |
| No 3A at all (raw V4L2 capture only) | 30 | **0** |
| `rkaiq_3A_server` killed and started fresh, then 2 s wait, before every stream start | 30, then 3 of a 100-cycle soak | **0**, then **1** (≈1/33 overall) |
| In-process rkaiq (uAPI2 init/prepare/start before `RK_MPI_VI`, as `sample_vi`) | 2 | **1** |

Ruled out: NPU activity, the hardware JPEG encoder (not linked), eMMC host runtime PM
(`power/control=on` → still failed), IR-cut switching (no IR-cut node in the DT, nothing in the 3A log),
3A writing to storage (it holds no files on eMMC), worn flash (EXT_CSD above).

## Reproduction

1. Buildroot image for the Pico Ultra eMMC config; SC3336 camera.
2. Start `rkaiq_3A_server`, then an application that opens `rkisp_mainpath` (`/dev/video11`, NV12,
   960x540) and streams.
3. Repeatedly stop the application (STREAMOFF + close) and start it again within ~5 s, with
   `rkaiq_3A_server` left running. Keep some write activity on the eMMC (e.g. `fsync` a small file
   every 5–10 s).
4. Within ~2–10 restarts the eMMC hangs as above.

## Controller state at the hang

Captured by a watchdog supervisor that reads `/sys/kernel/debug/mmc0/*` the moment the eMMC stops
completing writes (before the core re-initialises the card):

```
req: CMD25 (WRITE_MULTIPLE_BLOCK) ... DATA 0 / 32 * 512, err -115 (-EINPROGRESS); CMD12 pending
STATUS  0x0000cd06   FIFO empty, data state machine busy, card NOT busy
RINTSTS 0x00000010   TXDR (transmit FIFO data request) only
CTRL    0x02000030   INTMASK 0x0000b7cf   CLKENA 0x00010001
```

Full controller register dump at the hang (read through `/dev/mem` by the supervisor; persistent-3A
configuration; identical in all 3 hangs of a 30-cycle run, at cycles 5, 15 and 30):

```
CTRL    0x02000030   BMOD    0x00000682 (IDMAC on, fixed burst, PBL 128)
FIFOTH  0x607f0080   BYTCNT  0x0000b000   TCBCNT 0x00000000   TBBCNT 0x00000000
IDSTS   0x00004000   FSM = 2 (DESC_RD), no error bits (no FBE, no DU, no AIS)
DBADDR  0x0f612000   DSCADDR 0x0f612000   BUFADDR 0x00000000
```

The IDMAC is stuck **fetching its first descriptor** (`DSCADDR == DBADDR`, `BUFADDR` never loaded,
no bytes transferred). The read was issued on the bus and never completed, and no bus error was
reported. The card is idle. The descriptor ring sits in the 10 MB CMA pool at 0x0f600000, which is
also where `rkisp_hw` allocates its buffers ("No reserved memory region. default cma area!").
Sampling `clk_summary` across a restart showed **no clock rate changes**, and this kernel has no DDR
devfreq.

### Related code in the SDK kernel

`drivers/mmc/host/dw_mmc.c` and `dw_mmc-rockchip.c` contain an RV1106-specific workaround
(`is_rv1106_sd`, `g_sdmmc_ispvicap_lock`). The ISP (`isp/common.c`, `isp/hw.c`: writes to ISP
registers <= 0x200, `CIF_IRCL` reset, `CIF_ICCL` / `CTRL_VI_ISP_CLK_CTRL` clock gating) and VICAP
(`cif/capture.c`, `cif/subdev-itf.c`) drivers serialise against the SD host through it. The SD write
path is also changed (`BYTCNT=0`, a reserved dummy descriptor, IDMAC kept enabled on stop). It is
enabled only for `ffaa0000.mmc` and only when `rockchip_get_cpu_version() == 0`.

On this board:
- `OS_REG1` (0xff020204) = 1, so the cpu version is 1 and the workaround is off.
- The eMMC host `ffa90000.mmc` never gets it on any revision.
- The kernel is uniprocessor with `PREEMPT_NONE`.

**Questions:**
1. Does the ISP/VICAP <-> SDMMC erratum this workaround addresses also affect the **eMMC**
   controller (`ffa90000`)?
2. Is it really fixed in revision 1 silicon?

Our capture (an eMMC IDMAC write stalled in DESC_RD shortly after ISP reconfiguration) looks like the
same class of problem.

## Mitigation in use (not a fix)

Restarting `rkaiq_3A_server` (kill, wait for exit, start, wait 2 s) before every stream start
reduces the rate (≈1/33 restarts instead of ≈1/5) but does not remove it. The board recovers through
a hardware-watchdog reset (~45 s). The cause inside rkaiq (or between rkaiq/ISP activity and the
dw_mmc eMMC controller's DMA) is not visible to us — rkaiq is closed source — so we'd appreciate a
look at what rkaiq/ISP does ~5.5 s after a stream restart that could stall the eMMC IDMAC (DDR
frequency scaling, power-domain switching, or AXI/QoS changes would fit the timing).
