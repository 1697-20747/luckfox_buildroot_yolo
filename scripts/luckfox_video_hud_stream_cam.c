/* Persistent video-file -> YOLOv8n detection -> HUD-style overlay ->
 * MJPEG-over-HTTP live stream, for the Luckfox Pico Ultra (RV1106).
 *
 * "HUD-style" here means: always-visible readout panel (FPS, frame
 * count, detection count) plus labeled bounding boxes -- inspired by
 * apex_ros's apex_hud conventions (an always-visible instrument panel,
 * not "nothing drawn when no data"), NOT a literal port of apex_hud's
 * ROS 2 / rclpy widget library, which has no equivalent runtime here.
 *
 * Architecture: the model loads ONCE (rknn_init + zero-copy I/O set up
 * once, reused every frame -- re-doing that per frame would be far too
 * slow). An `ffmpeg` subprocess (spawned via popen) decodes/scales the
 * input to raw RGB24 frames at DISPLAY_W x DISPLAY_H (the source's own
 * resolution by default -- full quality for streaming/HUD, per 1697-20747's
 * "full resolution can go to streaming output and HUD, with reduced
 * resolution for inference"). Each frame: a cheap in-process bilinear
 * downscale to a SEPARATE MODEL_SIZE x MODEL_SIZE copy feeds the NPU
 * (inference cost is therefore unchanged regardless of display
 * resolution -- it's always run on the same small copy); detections
 * decode in that small copy's coordinate space (compute_dfl +
 * process_i8_rv1106 math, same as luckfox_yolov8_detect.c, restricted
 * to ALLOWED_CLASSES to cut CPU-side decode-loop cost) -> NMS -> box
 * coordinates rescaled back up to the FULL-res frame -> drawn there
 * (Rockchip's own image_drawing.c/font.h, real bitmap-font text
 * rendering, not written from scratch) -> JPEG-encoded at full
 * resolution (stb_image_write) -> pushed as one MJPEG multipart chunk
 * to a single connected HTTP client. Loops the video when it ends
 * (continuous "live" demo).
 *
 * Real tradeoff, measured not assumed: full-res streaming means more
 * pixels to JPEG-encode and send over the network per frame -- already
 * this pipeline's single biggest measured cost at 640x640 (~90ms/frame,
 * bigger than the ~67ms NPU inference itself). Displaying at full
 * source resolution will push real fps DOWN further, not up -- report
 * whatever the real measured number is, don't assume this trade is
 * free.
 *
 * Usage: sudo ./luckfox_video_hud_stream <model.rknn> <video-file-or-/dev/videoN> <labels.txt> [http-port] [display_w] [display_h]
 * display_w/display_h default to the source video's own resolution
 * (probed via ffprobe) if omitted, or to 1920x1080 for a live camera.
 * Then open http://<board-ip>:<port>/ in a browser on the same WiFi, or
 * over USB (the board's usb0 gadget-network interface, one cable for
 * both power and data) -- single-client by design (see README.md's
 * "Connectivity" note, 2026-09-20): no multi-client fan-out, no router.
 *
 * Live camera (2026-09-11): if the second argument starts with
 * "/dev/video" it's treated as a real V4L2 capture device (the
 * `rkisp_mainpath` node, e.g. /dev/video11) instead of a video file --
 * real, on-device verified sensor is a SmartSens sc3336 (3MP, real max
 * 2304x1296), see PROJECT_MEMORY.md for the full bring-up. Captured as
 * NV12 (the sensor/ISP's real native output format, confirmed via
 * `v4l2-ctl --list-formats-ext`) via a standard mmap capture loop, then
 * converted to RGB24 in-process (same BT.601 integer formula manually
 * verified against a real captured frame this session) to feed the rest
 * of this already-RGB24-based pipeline unchanged. Opened/configured
 * ONCE like the RKNN model -- unlike the video-file path, the V4L2
 * stream is NOT torn down between client (re)connections, since
 * reconfiguring a live sensor pipeline per HTTP client would be real,
 * unnecessary churn; frames simply go undrained (real, ordinary V4L2
 * driver behavior, not a crash risk) while no client is connected.
 *
 * Accelerometer HUD readout (MPU-6050, real device-tree/kernel fix
 * written and build-in-progress as of this addition -- NOT yet
 * confirmed live on real hardware, see PROJECT_MEMORY.md): reads the
 * standard Linux IIO sysfs interface the mainline inv_mpu6050 driver
 * exposes (/sys/bus/iio/devices/iio:deviceN/in_accel_{x,y,z}_raw +
 * in_accel_scale), auto-discovering deviceN by matching the "name"
 * file to "mpu6050" rather than hardcoding an index. Per 1697-20747's
 * explicit ask ("add smoothing averaging filter... to prevent noisy
 * signal from accelerometer... jumpy visuals"): raw readings pass
 * through a simple exponential moving average (EMA) per axis before
 * being drawn -- real, standard technique for exactly this problem
 * (a low-pass filter trading responsiveness for stability; ACCEL_EMA_ALPHA
 * below controls that tradeoff, not tuned against real sensor noise
 * yet since no real data has been read from actual hardware). Real
 * achieved sample rate is measured and printed periodically, same
 * "report the honest number" discipline as the video fps stats --
 * don't assume a rate, read it. Follows this file's own established
 * "always draw the instrument, neutral/grey when no data" convention:
 * the panel always renders, showing "ACCEL: not connected" if the IIO
 * device isn't found rather than omitting the readout or crashing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <linux/watchdog.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <linux/videodev2.h>
#include <arm_neon.h>
#include <linux/dma-buf.h>
#include <sys/un.h>
#include <stddef.h>
#include "yolo_camd.h"
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <functional>

extern "C" {
#include "utils/image_drawing.h"
#include "apex_hud_widgets.h"
}

#include <drm/drm.h>
#include <drm/drm_mode.h>

/* libjpeg-turbo (already the real backing implementation of this
 * board's installed libjpeg.so.8 -- confirmed via `dpkg -l`, apt
 * package `libjpeg-turbo8`) replaces stb_image_write's JPEG encoder
 * here. Real, measured motivation: JPEG encode is this pipeline's
 * single biggest cost (~146-165ms/frame at 1280x720, confirmed by
 * splitting it from network-send time -- see PROJECT_MEMORY.md), and
 * stb_image_write's encoder is a simple single-pass implementation
 * with no SIMD, while libjpeg-turbo has hand-written ARM NEON paths
 * (this chip's `-mfpu=neon` is real and otherwise unused) for color
 * conversion and DCT. `libjpeg-turbo8-dev` installed via apt for
 * jpeglib.h -- not built from source, a real existing package. */
#include <jpeglib.h>
#include <jerror.h>

#include "rknn_api.h"
#include "im2d.h"

/* Hardware JPEG encode via Rockchip's Rockit media API (RK_MPI_VENC), the
 * path rkipc and the SDK samples use on RV1106. Only in the Buildroot build
 * (tools/build_app.sh defines LUCKFOX_HW_JPEG and links librockit); the
 * Ubuntu build keeps libjpeg-turbo, where the vendor media stack isn't
 * usable (raw MPP oopsed the kernel driver, see PROJECT_MEMORY.md). */
#ifdef LUCKFOX_ROCKIT_VI
/* Option C (2026-10-11): camera capture + 3A the way Rockchip's own RV1106
 * software does it (rkipc, sample_vi) -- all in one process through Rockit's
 * VI and in-process rkaiq (uAPI2) -- instead of raw V4L2 next to a separate
 * rkaiq_3A_server, the mix implicated in the eMMC failures. Built as a
 * separate binary (luckfox_video_hud_stream_rockit); selected with
 * LUCKFOX_PIPELINE=rockit in app.env, falling back to the V4L2 build. */
#include "rk_aiq_user_api2_sysctl.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_vi.h"
#endif
#ifdef LUCKFOX_HW_JPEG
#include "rk_mpi_sys.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_venc.h"
#endif

#define OBJ_CLASS_NUM 80
#define NUM_SCALES 3
#define BOX_THRESH 0.25f
#define NMS_THRESH 0.45f
#define MODEL_SIZE 640   /* fallback only: the real input size is read from the model */
/* Model input size, from the .rknn's own input tensor at start-up. A 16:9
 * model (e.g. 800x448, tools/export_yolov8_rknn.sh) sees the camera frame with
 * its proportions intact; the original 640x640 model gets it stretched. */
static int g_model_w = MODEL_SIZE, g_model_h = MODEL_SIZE;

/* Rolling average over the last ROLL_N frames for /api/status: a single
 * frame's timing is noisy on this one-core board (a frame preempted by
 * another thread can take 2-3x the typical time). */
#define ROLL_N 30
typedef struct { double v[ROLL_N]; int n, i; double sum; } RollAvg;
static double roll_add(RollAvg* r, double x) {
    if (r->n == ROLL_N) r->sum -= r->v[r->i]; else r->n++;
    r->v[r->i] = x; r->sum += x; r->i = (r->i + 1) % ROLL_N;
    return r->sum / r->n;
}
static RollAvg g_roll_infer, g_roll_read, g_roll_jpeg;
#define HTTP_PORT_DEFAULT 8080

static const int STRIDES[NUM_SCALES] = {8, 16, 32};

/* 1697-20747: "object types can be restricted for now to speed inference as
 * needed" -- COCO class ids relevant to the linked test videos
 * (bikes/cars/motorbikes/people): person=0, bicycle=1, car=2,
 * motorcycle=3, bus=5, truck=7. Skipping the other 74 classes in the
 * decode loop cuts real CPU work per grid cell -- edit this set to
 * widen/narrow what gets detected. */
static std::set<int> ALLOWED_CLASSES = {0, 1, 2, 3, 5, 7};
/* The same list as a plain array for the per-cell decode loop: 80 std::set
 * lookups per grid cell cost ~20 ms per frame on the 800x448 model, where the
 * score-sum pre-filter no longer skips most cells. */
static const int ALLOWED_LIST[] = {0, 1, 2, 3, 5, 7};

/* --- MPU-6050 accelerometer via Linux IIO sysfs, with EMA smoothing --- */

/* Real, standard low-pass filter: smoothed = alpha*new + (1-alpha)*old.
 * Lower alpha = smoother/slower to respond, higher alpha = more
 * responsive/noisier. Not tuned against real sensor noise yet -- no
 * real data has been read from actual hardware as of this addition.
 * 0.2 is a reasonable, commonly-used starting point for this exact
 * kind of "visual HUD readout" use case (not a safety-critical control
 * loop, where a much smaller alpha or a proper complementary/Kalman
 * filter would be worth the extra complexity). */
#define ACCEL_EMA_ALPHA 0.2f

typedef struct {
    bool found;
    char device_path[256];   /* /sys/bus/iio/devices/iio:deviceN */
    float scale;              /* raw-to-g conversion, read once from in_accel_scale */
    float smoothed_x, smoothed_y, smoothed_z;
    bool ema_initialized;     /* first real reading seeds the EMA instead of starting at 0 */
    int reads_ok, reads_fail;
    struct timespec rate_t0;
    int rate_count;
    float measured_hz;
} AccelState;

/* Auto-discovers the mpu6050 IIO device by checking each iio:deviceN's
 * "name" file, rather than hardcoding an index (device numbering isn't
 * guaranteed stable across other IIO devices the kernel might expose,
 * e.g. the SoC's own ADC seen earlier in this project's real
 * /sys/bus/iio/devices/ check). */
static void accel_discover(AccelState* st) {
    memset(st, 0, sizeof(*st));
    for (int i = 0; i < 8; i++) {
        char name_path[256], name_buf[64];
        snprintf(name_path, sizeof(name_path), "/sys/bus/iio/devices/iio:device%d/name", i);
        FILE* f = fopen(name_path, "r");
        if (!f) continue;
        if (fgets(name_buf, sizeof(name_buf), f)) {
            name_buf[strcspn(name_buf, "\r\n")] = 0;
            if (strstr(name_buf, "mpu6050") != NULL) {
                snprintf(st->device_path, sizeof(st->device_path), "/sys/bus/iio/devices/iio:device%d", i);
                st->found = true;
                fclose(f);
                break;
            }
        }
        fclose(f);
    }
    if (!st->found) {
        printf("RESULT: ACCEL_NOT_FOUND (no iio device named 'mpu6050' -- check the kernel fix has been flashed and booted)\n");
        return;
    }
    char scale_path[300];
    snprintf(scale_path, sizeof(scale_path), "%s/in_accel_scale", st->device_path);
    FILE* sf = fopen(scale_path, "r");
    if (sf) {
        if (fscanf(sf, "%f", &st->scale) != 1) st->scale = 1.0f;
        fclose(sf);
    } else {
        st->scale = 1.0f; /* fall back to raw units if scale file is missing */
    }
    printf("RESULT: ACCEL_FOUND %s scale=%f\n", st->device_path, st->scale);
    clock_gettime(CLOCK_MONOTONIC, &st->rate_t0);
}

static bool accel_read_raw_axis(const char* device_path, const char* axis, float* out_g, float scale) {
    char path[300];
    snprintf(path, sizeof(path), "%s/in_accel_%s_raw", device_path, axis);
    FILE* f = fopen(path, "r");
    if (!f) return false;
    int raw;
    bool ok = fscanf(f, "%d", &raw) == 1;
    fclose(f);
    if (ok) *out_g = raw * scale;
    return ok;
}

/* Reads real x/y/z, applies the EMA filter, and tracks real achieved
 * sample rate (printed every 30 successful reads -- report the real
 * number once hardware exists, don't assume one). */
static void accel_update(AccelState* st) {
    if (!st->found) return;
    float x, y, z;
    bool ok = accel_read_raw_axis(st->device_path, "x", &x, st->scale) &&
              accel_read_raw_axis(st->device_path, "y", &y, st->scale) &&
              accel_read_raw_axis(st->device_path, "z", &z, st->scale);
    if (!ok) {
        st->reads_fail++;
        return;
    }
    st->reads_ok++;
    if (!st->ema_initialized) {
        st->smoothed_x = x; st->smoothed_y = y; st->smoothed_z = z;
        st->ema_initialized = true;
    } else {
        st->smoothed_x = ACCEL_EMA_ALPHA * x + (1 - ACCEL_EMA_ALPHA) * st->smoothed_x;
        st->smoothed_y = ACCEL_EMA_ALPHA * y + (1 - ACCEL_EMA_ALPHA) * st->smoothed_y;
        st->smoothed_z = ACCEL_EMA_ALPHA * z + (1 - ACCEL_EMA_ALPHA) * st->smoothed_z;
    }

    st->rate_count++;
    if (st->rate_count >= 30) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - st->rate_t0.tv_sec) + (now.tv_nsec - st->rate_t0.tv_nsec) / 1e9;
        st->measured_hz = elapsed > 0 ? st->rate_count / elapsed : 0;
        printf("ACCEL real sample rate: %.1f Hz over last %d reads (%d ok, %d fail total)\n",
               st->measured_hz, st->rate_count, st->reads_ok, st->reads_fail);
        st->rate_count = 0;
        st->rate_t0 = now;
    }
}

/* --- mounting calibration: "Set level" (2026-10-13) ---
 * The MPU-6050 gives tilt relative to gravity, absolute and drift-free, but in
 * the SENSOR's own frame: mounted at an angle, "level" reads as that angle.
 * The web UI's Set level button stores the gravity direction measured while
 * the unit stands level; every later reading is rotated by the rotation that
 * takes that stored direction onto straight down (+Z here), so pitch/roll are
 * then the unit's own -- correct even for large mounting angles, unlike
 * subtracting angle offsets. Saved to /userdata (tmp file + fsync + rename),
 * loaded at start-up. Deliberately NOT done automatically at boot: a board
 * switched on while tilted would call that tilt level. */
#define LEVEL_FILE "/userdata/accel-level.conf"
static bool g_level_set = false;
static float g_level_R[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
static AccelState* g_accel = NULL;   /* for the /api/level handler */

static bool level_from_vector(float gx, float gy, float gz) {
    float n = sqrtf(gx * gx + gy * gy + gz * gz);
    /* readings are in m/s^2 (IIO scale), despite the smoothed_* "g" naming;
     * only the direction is used. This module reads ~12.5 m/s^2 at rest (z
     * offset), so the window is wide: it only rejects a unit being moved. */
    if (n < 5.0f || n > 15.0f) return false;
    float v[3] = { gx / n, gy / n, gz / n };
    /* Rodrigues: rotate v onto t = (0, 0, 1); axis k = v x t */
    float k[3] = { v[1], -v[0], 0.0f };
    float sn = sqrtf(k[0] * k[0] + k[1] * k[1]);
    float c = v[2];
    float R[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    if (sn < 1e-6f) {
        if (c < 0) { R[1][1] = -1; R[2][2] = -1; }   /* upside down: 180 deg about X */
    } else {
        float K[3][3] = { { 0, -k[2], k[1] }, { k[2], 0, -k[0] }, { -k[1], k[0], 0 } };
        float f = (1 - c) / (sn * sn);
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++) {
                float kk = 0;
                for (int m = 0; m < 3; m++) kk += K[a][m] * K[m][b];
                R[a][b] += K[a][b] + f * kk;
            }
    }
    memcpy(g_level_R, R, sizeof(R));
    g_level_set = true;
    return true;
}

static void level_clear(void) {
    float I[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    memcpy(g_level_R, I, sizeof(I));
    g_level_set = false;
}

static void level_load(void) {
    FILE* f = fopen(LEVEL_FILE, "r");
    if (!f) return;
    float x, y, z;
    if (fscanf(f, "%f %f %f", &x, &y, &z) == 3 && level_from_vector(x, y, z))
        printf("RESULT: LEVEL_LOADED gravity reference (%.3f, %.3f, %.3f) m/s^2\n", x, y, z);
    fclose(f);
}

static bool level_save(float x, float y, float z) {
    const char* tmp = LEVEL_FILE ".tmp";
    FILE* f = fopen(tmp, "w");
    if (!f) return false;
    fprintf(f, "%.5f %.5f %.5f\n", x, y, z);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    if (rename(tmp, LEVEL_FILE) != 0) return false;
    int d = open("/userdata", O_RDONLY | O_CLOEXEC);
    if (d >= 0) { fsync(d); close(d); }
    return true;
}

/* pitch/roll in degrees from the smoothed gravity vector, after the mounting
 * correction */
static void accel_tilt(const AccelState* a, int* has_tilt, float* pitch, float* roll) {
    *has_tilt = a->found && a->ema_initialized;
    *pitch = *roll = 0;
    if (!*has_tilt) return;
    float g[3] = { a->smoothed_x, a->smoothed_y, a->smoothed_z }, r[3];
    for (int i = 0; i < 3; i++) r[i] = g_level_R[i][0] * g[0] + g_level_R[i][1] * g[1] + g_level_R[i][2] * g[2];
    *roll = atan2f(r[1], r[2]) * 180.0f / (float)M_PI;
    *pitch = atan2f(-r[0], sqrtf(r[1] * r[1] + r[2] * r[2])) * 180.0f / (float)M_PI;
}

/* Real system-wide CPU%, read from /proc/stat -- delta of (total-idle)
 * jiffies between calls, same standard technique `top` itself uses.
 * Returns 0.0 on the first call (no prior sample to delta against). */
static float read_cpu_percent(void) {
    static unsigned long long prev_total = 0, prev_idle = 0;
    static bool have_prev = false;

    FILE* f = fopen("/proc/stat", "r");
    if (!f) return 0.0f;
    unsigned long long user, nice, system_, idle, iowait, irq, softirq, steal;
    int n = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                   &user, &nice, &system_, &idle, &iowait, &irq, &softirq, &steal);
    fclose(f);
    if (n < 4) return 0.0f;

    unsigned long long idle_all = idle + iowait;
    unsigned long long total = user + nice + system_ + idle_all + irq + softirq + steal;

    float pct = 0.0f;
    if (have_prev && total > prev_total) {
        unsigned long long total_delta = total - prev_total;
        unsigned long long idle_delta = idle_all - prev_idle;
        pct = 100.0f * (float)(total_delta - idle_delta) / (float)total_delta;
    }
    prev_total = total;
    prev_idle = idle_all;
    have_prev = true;
    return pct;
}

static char class_names[OBJ_CLASS_NUM][64];
static unsigned int class_colors[OBJ_CLASS_NUM];

static void load_labels(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "WARNING: no labels file, using indices\n"); }
    for (int i = 0; i < OBJ_CLASS_NUM; i++) {
        if (f && fgets(class_names[i], 64, f)) {
            class_names[i][strcspn(class_names[i], "\r\n")] = 0;
        } else {
            snprintf(class_names[i], 64, "class_%d", i);
        }
        /* deterministic distinct-ish color per class */
        unsigned int h = (unsigned int)(i * 2654435761u);
        class_colors[i] = 0xFF000000u | ((h & 0xFF) << 16) | (((h >> 8) & 0xFF) << 8) | ((h >> 16) & 0xFF);
    }
    if (f) fclose(f);
}

static float deqnt(int8_t q, int32_t zp, float scale) { return ((float)q - (float)zp) * scale; }
static int8_t qnt(float f, int32_t zp, float scale) {
    float v = (f / scale) + zp;
    if (v < -128) v = -128;
    if (v > 127) v = 127;
    return (int8_t)v;
}

typedef struct { float x, y, w, h, score; int cls_id; } Detection;

static void compute_dfl(float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; b++) {
        std::vector<float> exp_t(dfl_len);
        float exp_sum = 0, acc_sum = 0;
        for (int i = 0; i < dfl_len; i++) { exp_t[i] = expf(tensor[i + b * dfl_len]); exp_sum += exp_t[i]; }
        for (int i = 0; i < dfl_len; i++) acc_sum += exp_t[i] / exp_sum * i;
        box[b] = acc_sum;
    }
}

static void decode_scale(int8_t* box_t, int32_t box_zp, float box_scale,
                          int8_t* score_t, int32_t score_zp, float score_scale,
                          int8_t* score_sum_t, int32_t score_sum_zp, float score_sum_scale,
                          int grid_h, int grid_w, int stride, int dfl_len,
                          std::vector<Detection>& out) {
    int8_t score_thres_i8 = qnt(BOX_THRESH, score_zp, score_scale);
    int8_t score_sum_thres_i8 = qnt(BOX_THRESH, score_sum_zp, score_sum_scale);
    for (int i = 0; i < grid_h; i++) {
        for (int j = 0; j < grid_w; j++) {
            int offset = i * grid_w + j;
            if (score_sum_t != NULL && score_sum_t[offset] < score_sum_thres_i8) continue;
            int max_class_id = -1;
            int8_t max_score = -128;
            int score_offset = offset * OBJ_CLASS_NUM;
            /* Quick reject with NEON: the max over all 80 class scores, 16 per
             * instruction. The score-sum pre-filter above can't reject on its
             * own when the sum's no-object floor is already over the threshold
             * (800x448 model, stride 8: 80 tiny class scores add up to ~0.27),
             * and the scalar scan of every cell then cost ~20 ms per frame. */
            if (ALLOWED_CLASSES.empty() && OBJ_CLASS_NUM == 80) {
                const int8_t* sp = score_t + score_offset;
                int8x16_t m = vmaxq_s8(vmaxq_s8(vld1q_s8(sp), vld1q_s8(sp + 16)),
                                       vmaxq_s8(vmaxq_s8(vld1q_s8(sp + 32), vld1q_s8(sp + 48)), vld1q_s8(sp + 64)));
                int8x8_t m8 = vpmax_s8(vget_low_s8(m), vget_high_s8(m));
                m8 = vpmax_s8(m8, m8); m8 = vpmax_s8(m8, m8); m8 = vpmax_s8(m8, m8);
                if (vget_lane_s8(m8, 0) <= score_thres_i8) continue;
            }
            if (!ALLOWED_CLASSES.empty()) {
                for (int c : ALLOWED_LIST) {
                    if (score_t[score_offset + c] > score_thres_i8 && score_t[score_offset + c] > max_score) {
                        max_score = score_t[score_offset + c];
                        max_class_id = c;
                    }
                }
            } else {
                for (int c = 0; c < OBJ_CLASS_NUM; c++) {
                    if (score_t[score_offset + c] > score_thres_i8 && score_t[score_offset + c] > max_score) {
                        max_score = score_t[score_offset + c];
                        max_class_id = c;
                    }
                }
            }
            if (max_score <= score_thres_i8) continue;
            int box_offset = (i * grid_w + j) * 4 * dfl_len;
            std::vector<float> before_dfl(dfl_len * 4);
            for (int k = 0; k < dfl_len * 4; k++) before_dfl[k] = deqnt(box_t[box_offset + k], box_zp, box_scale);
            float box[4];
            compute_dfl(before_dfl.data(), dfl_len, box);
            float x1 = (-box[0] + j + 0.5f) * stride, y1 = (-box[1] + i + 0.5f) * stride;
            float x2 = (box[2] + j + 0.5f) * stride, y2 = (box[3] + i + 0.5f) * stride;
            Detection d; d.x = x1; d.y = y1; d.w = x2 - x1; d.h = y2 - y1;
            d.score = deqnt(max_score, score_zp, score_scale); d.cls_id = max_class_id;
            out.push_back(d);
        }
    }
}

static float iou(const Detection& a, const Detection& b) {
    float xmin0 = a.x, ymin0 = a.y, xmax0 = a.x + a.w, ymax0 = a.y + a.h;
    float xmin1 = b.x, ymin1 = b.y, xmax1 = b.x + b.w, ymax1 = b.y + b.h;
    float w = fmaxf(0.f, fminf(xmax0, xmax1) - fmaxf(xmin0, xmin1) + 1.f);
    float h = fmaxf(0.f, fminf(ymax0, ymax1) - fmaxf(ymin0, ymin1) + 1.f);
    float inter = w * h;
    float uni = (xmax0 - xmin0 + 1.f) * (ymax0 - ymin0 + 1.f) + (xmax1 - xmin1 + 1.f) * (ymax1 - ymin1 + 1.f) - inter;
    return uni <= 0.f ? 0.f : inter / uni;
}

static std::vector<Detection> nms(std::vector<Detection>& dets) {
    std::sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) { return a.score > b.score; });
    std::vector<bool> removed(dets.size(), false);
    std::vector<Detection> keep;
    for (size_t i = 0; i < dets.size(); i++) {
        if (removed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); j++) {
            if (removed[j] || dets[j].cls_id != dets[i].cls_id) continue;
            if (iou(dets[i], dets[j]) > NMS_THRESH) removed[j] = true;
        }
    }
    return keep;
}

/* Cheap nearest-neighbor downscale from the full display-resolution
 * frame to a separate MODEL_SIZE x MODEL_SIZE copy for NPU input --
 * keeps inference cost constant regardless of display/stream
 * resolution (1697-20747: "full resolution can go to streaming output and
 * hud, with reduced resolution for inference on chip"). */
/* Nearest-neighbour, with the source column offsets computed once per size:
 * with the model size known only at run time (read from the .rknn), the old
 * per-pixel x * src_w / dst_w became a real hardware divide per pixel --
 * 27 ms per 800x448 frame on the A7 instead of ~10 ms. */
static void downscale_for_inference(unsigned char* src, int src_w, int src_h, unsigned char* dst, int dst_w, int dst_h) {
    static int* xoff = NULL;
    static int lut_src_w = 0, lut_dst_w = 0;
    if (lut_src_w != src_w || lut_dst_w != dst_w) {
        free(xoff);
        xoff = (int*)malloc(sizeof(int) * dst_w);
        for (int x = 0; x < dst_w; x++) xoff[x] = (x * src_w / dst_w) * 3;
        lut_src_w = src_w; lut_dst_w = dst_w;
    }
    for (int y = 0; y < dst_h; y++) {
        const unsigned char* src_row = src + (size_t)(y * src_h / dst_h) * src_w * 3;
        unsigned char* d = dst + (size_t)y * dst_w * 3;
        for (int x = 0; x < dst_w; x++, d += 3) {
            const unsigned char* p = src_row + xoff[x];
            d[0] = p[0]; d[1] = p[1]; d[2] = p[2];
        }
    }
}

/* --- DRM "dumb buffer" allocator: real, physically-contiguous DMA
 * memory for the RGA hardware path.
 *
 * Real, on-device finding: RV1106's RGA (rga2, hw_version 3.3.87975 per
 * dmesg) has NO IOMMU (kernel logs this literally: "rga_mm: Current
 * NONE_MMU[0] cannot support physically discontinuous virtual
 * address!"), confirmed by a real failed submit when this pipeline
 * first tried handing it a plain malloc()'d buffer -- ordinary heap
 * memory is virtually contiguous but not physically contiguous, so the
 * driver rejected it outright (job_commit failed, not a crash, but no
 * frames could be produced). The real fix, not a workaround: allocate
 * frame_full/frame_small as DRM "dumb buffers" instead of malloc() --
 * these ARE physically contiguous by construction. Uses raw DRM ioctls
 * directly against /dev/dri/card0 (present and confirmed working on
 * this board) rather than linking/dlopen-ing libdrm.so, matching the
 * exact real ioctl sequence Rockchip's own librga sample allocator
 * (samples/utils/allocator/drm_alloc.cpp, fetched from
 * github.com/airockchip/librga for reference) uses, since this Ubuntu
 * image ships only libdrm's runtime .so (no dev symlink/headers) --
 * this board's own /usr/include/drm/{drm,drm_mode}.h (kernel uapi
 * headers, already present) are enough for the raw ioctl calls, no
 * libdrm dependency needed at all. */
static int g_drm_fd = -1;

static bool drm_init() {
    g_drm_fd = open("/dev/dri/card0", O_RDWR);
    if (g_drm_fd < 0) { perror("open /dev/dri/card0"); return false; }
    return true;
}

/* Returns the mmap'd virtual address of a new contiguous buffer, and
 * reports the driver's real byte pitch (out_stride_px, converted to
 * pixels for the given bytes-per-pixel) -- NOT assumed to equal width,
 * since DRM dumb-buffer allocators are free to pad each row for
 * alignment; RGA needs the real stride or a padded buffer would decode
 * as a skewed/torn image. */
static void* drm_alloc_contig(int w, int h, int bytes_per_px, int* out_stride_px) {
    if (g_drm_fd < 0) return NULL;
    struct drm_mode_create_dumb create_arg;
    memset(&create_arg, 0, sizeof(create_arg));
    create_arg.width = (uint32_t)w;
    create_arg.height = (uint32_t)h;
    create_arg.bpp = (uint32_t)(bytes_per_px * 8);
    if (ioctl(g_drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_arg) < 0) {
        perror("DRM_IOCTL_MODE_CREATE_DUMB"); return NULL;
    }
    struct drm_mode_map_dumb map_arg;
    memset(&map_arg, 0, sizeof(map_arg));
    map_arg.handle = create_arg.handle;
    if (ioctl(g_drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map_arg) < 0) {
        perror("DRM_IOCTL_MODE_MAP_DUMB"); return NULL;
    }
    void* addr = mmap(0, create_arg.size, PROT_READ | PROT_WRITE, MAP_SHARED, g_drm_fd, map_arg.offset);
    if (addr == MAP_FAILED) { perror("mmap drm dumb buffer"); return NULL; }
    *out_stride_px = (int)(create_arg.pitch / bytes_per_px);
    printf("RESULT: DRM_BUF_ALLOC %dx%d pitch=%u stride_px=%d size=%llu\n",
           w, h, create_arg.pitch, *out_stride_px, (unsigned long long)create_arg.size);
    return addr;
}

static int g_src_stride_px = 0, g_dst_stride_px = 0;
static unsigned char* g_drm_src = NULL;   /* display_w x display_h x 3, DRM-backed, RGA src only */
static unsigned char* g_drm_dst = NULL;   /* MODEL_SIZE x MODEL_SIZE x 3, DRM-backed, RGA dst only */

/* Same contract as downscale_for_inference() above, but offloaded to
 * RV1106's real 2D Graphics Engine (RGA, "rga2" per dmesg hw_version
 * 3.3.87975) instead of a CPU nearest-neighbor loop -- confirmed via
 * real device discovery (ldconfig -p) that a glibc-built librga.so.2
 * (the modern im2d API, rga_api version 1.10.1) is present in /usr/lib
 * on this board's Ubuntu image, unlike MPP (only a uClibc build exists,
 * see RV1106_NPU_YOLO_GUIDE.md).
 *
 * Real, measured, non-obvious finding that shaped this function's
 * design: RV1106's DRM driver maps "dumb buffer" memory (required for
 * RGA -- see drm_alloc_contig's comment for the no-IOMMU root cause)
 * as CPU-uncached. Fine for RGA's own hardware DMA access, genuinely
 * bad for CPU access: a first attempt that made frame_full/frame_small
 * themselves DRM-backed (so RGA could touch them with zero copies) cut
 * downscale_ms from ~20ms to ~2.5ms as expected, but every OTHER
 * CPU-heavy stage that touches those same buffers (the quantization
 * loop, HUD drawing, JPEG encode) got dramatically slower reading
 * through uncached memory -- infer_ms 85ms->323ms, jpeg_ms 145ms->
 * 548ms, a large NET LOSS (fps dropped 3.3->1.0), a real result kept
 * here rather than quietly discarded. Fix: keep frame_full/frame_small
 * as ordinary cached malloc() buffers (fast for CPU work, as before),
 * and bounce through two small persistent DRM buffers ONLY around the
 * RGA call itself -- memcpy in (cached read, WC/uncached write --
 * write-combining hardware coalesces sequential writes cheaply even
 * uncached).
 *
 * A SECOND real attempt tried removing the memcpy-out too -- having
 * the caller's quantization loop read g_drm_dst directly instead of a
 * copy into frame_small, since that loop already touches every dst
 * byte once anyway. Measured result: much WORSE (infer_ms 85ms->
 * ~325ms), not better -- a scalar per-byte cast loop against uncached
 * memory pays the full uncached-access penalty on every single byte
 * access, where a bulk memcpy() apparently doesn't (likely a
 * vectorized/wide-load libc implementation issuing far fewer, wider
 * bus transactions). Real, somewhat counter-intuitive lesson: on
 * uncached memory, "one fewer pass" is not automatically faster than
 * "one extra memcpy" -- ACCESS PATTERN matters as much as pass count.
 * Kept the memcpy-out below because of this real measurement, not
 * removed for being "obviously" redundant. */
static bool downscale_for_inference_rga(unsigned char* src, int src_w, int src_h, unsigned char* dst, int dst_w, int dst_h) {
    memcpy(g_drm_src, src, (size_t)src_w * src_h * 3);
    rga_buffer_t src_buf = wrapbuffer_virtualaddr_t(g_drm_src, src_w, src_h, g_src_stride_px, src_h, RK_FORMAT_RGB_888);
    rga_buffer_t dst_buf = wrapbuffer_virtualaddr_t(g_drm_dst, dst_w, dst_h, g_dst_stride_px, dst_h, RK_FORMAT_RGB_888);
    IM_STATUS ret = imresize(src_buf, dst_buf);
    if (ret != IM_STATUS_SUCCESS) return false;
    memcpy(dst, g_drm_dst, (size_t)dst_w * dst_h * 3);
    return true;
}

/* image_drawing.c's draw_rectangle only draws outlines -- a small
 * solid-fill helper for the HUD readout panel's background. */
static void fill_rect(unsigned char* rgb, int img_w, int img_h, int x, int y, int w, int h, unsigned char r, unsigned char g, unsigned char b) {
    for (int yy = y; yy < y + h && yy < img_h; yy++) {
        if (yy < 0) continue;
        for (int xx = x; xx < x + w && xx < img_w; xx++) {
            if (xx < 0) continue;
            int idx = (yy * img_w + xx) * 3;
            rgb[idx] = r; rgb[idx + 1] = g; rgb[idx + 2] = b;
        }
    }
}

/* --- live V4L2 camera capture: rkisp_mainpath, NV12, standard mmap
 * capture loop --- real, on-device confirmed pipeline this session
 * (see PROJECT_MEMORY.md's 2026-09-11 camera-bring-up entry): the
 * sensor is a real sc3336 bound to its own kernel driver, `rkcif`/
 * `rkisp` are loaded and active, and `/dev/video11`-`18` (the
 * `rkisp_mainpath` capture nodes) report real, working
 * V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE capture (confirmed via
 * `v4l2-ctl --list-formats-ext` and a real `--stream-mmap` grab whose
 * exact byte count and pixel content were checked, not just a clean
 * ioctl exit code). */
#define V4L2_NUM_BUFFERS 4

typedef struct {
    int fd;
    int width, height;   /* real negotiated size -- may differ from what was requested */
    struct { void* start; size_t length; } buffers[V4L2_NUM_BUFFERS];
    int n_buffers;
} V4L2Capture;

static bool v4l2_open_and_configure(V4L2Capture* cap, const char* dev_path, int want_w, int want_h) {
    memset(cap, 0, sizeof(*cap));
    cap->fd = open(dev_path, O_RDWR);
    if (cap->fd < 0) { perror("open v4l2 device"); return false; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = want_w;
    fmt.fmt.pix_mp.height = want_h;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (ioctl(cap->fd, VIDIOC_S_FMT, &fmt) < 0) { perror("VIDIOC_S_FMT"); return false; }
    /* driver may adjust to the nearest supported stepwise size -- use
     * what it actually negotiated, not what was requested */
    cap->width = fmt.fmt.pix_mp.width;
    cap->height = fmt.fmt.pix_mp.height;

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = V4L2_NUM_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(cap->fd, VIDIOC_REQBUFS, &req) < 0) { perror("VIDIOC_REQBUFS"); return false; }
    cap->n_buffers = (int)req.count;

    for (int i = 0; i < cap->n_buffers; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane plane;
        memset(&buf, 0, sizeof(buf));
        memset(&plane, 0, sizeof(plane));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = &plane;
        buf.length = 1;
        if (ioctl(cap->fd, VIDIOC_QUERYBUF, &buf) < 0) { perror("VIDIOC_QUERYBUF"); return false; }
        cap->buffers[i].length = plane.length;
        cap->buffers[i].start = mmap(NULL, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, cap->fd, plane.m.mem_offset);
        if (cap->buffers[i].start == MAP_FAILED) { perror("mmap v4l2 buffer"); return false; }
        if (ioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) { perror("VIDIOC_QBUF initial"); return false; }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(cap->fd, VIDIOC_STREAMON, &type) < 0) { perror("VIDIOC_STREAMON"); return false; }

    printf("RESULT: V4L2_CAPTURE_READY %s %dx%d NV12 buffers=%d\n", dev_path, cap->width, cap->height, cap->n_buffers);
    return true;
}

/* NV12 -> RGB888, BT.601 integer conversion -- same formula manually
 * verified this session against a real captured frame (decoded by hand
 * in Python, checked visually) before trusting it here. Standard NV12
 * layout: full-res Y plane, followed by a half-res/half-res interleaved
 * U/V plane. */
static void nv12_to_rgb(const unsigned char* nv12, int w, int h, unsigned char* rgb) {
    const unsigned char* y_plane = nv12;
    const unsigned char* uv_plane = nv12 + (size_t)w * h;
    for (int row = 0; row < h; row++) {
        const unsigned char* y_row = y_plane + (size_t)row * w;
        const unsigned char* uv_row = uv_plane + (size_t)(row / 2) * w;
        unsigned char* out_row = rgb + (size_t)row * w * 3;
        for (int col = 0; col < w; col++) {
            int c = y_row[col] - 16;
            int uv_col = (col / 2) * 2;
            int u = uv_row[uv_col] - 128;
            int v = uv_row[uv_col + 1] - 128;
            int r = (298 * c + 409 * v + 128) >> 8;
            int g = (298 * c - 100 * u - 208 * v + 128) >> 8;
            int b = (298 * c + 516 * u + 128) >> 8;
            if (r < 0) r = 0; else if (r > 255) r = 255;
            if (g < 0) g = 0; else if (g > 255) g = 255;
            if (b < 0) b = 0; else if (b > 255) b = 255;
            out_row[col * 3 + 0] = (unsigned char)r;
            out_row[col * 3 + 1] = (unsigned char)g;
            out_row[col * 3 + 2] = (unsigned char)b;
        }
    }
}

/* Dequeue one real captured frame, convert it in place into out_rgb
 * (must be cap->width x cap->height x 3), and requeue the buffer for
 * the driver to fill again. Returns false on a real V4L2 error only --
 * unlike the video-file path, there's no "end of stream" case here. */
static bool v4l2_capture_frame(V4L2Capture* cap, unsigned char* out_rgb) {
    struct v4l2_buffer buf;
    struct v4l2_plane plane;
    memset(&buf, 0, sizeof(buf));
    memset(&plane, 0, sizeof(plane));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = &plane;
    buf.length = 1;
    if (ioctl(cap->fd, VIDIOC_DQBUF, &buf) < 0) { perror("VIDIOC_DQBUF"); return false; }
    nv12_to_rgb((const unsigned char*)cap->buffers[buf.index].start, cap->width, cap->height, out_rgb);
    if (ioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) { perror("VIDIOC_QBUF requeue"); return false; }
    return true;
}

/* --- frames from yolo_camd (default): the camera stream lives in that
 * process for the whole boot and is never stopped, so this server can exit
 * and restart without restarting the stream (every eMMC hang on this board
 * followed a stream restart -- see scripts/yolo_camd.c). Its capture buffers
 * arrive as DMA-BUF fds and are mapped once; per frame we borrow the newest
 * one, convert it, and hand it back. LUCKFOX_CAMD=0: capture directly. --- */
typedef struct {
    int sock;
    int n;
    struct { int fd; void* start; size_t length; } buffers[CAMD_MAX_BUFFERS];
} CamdClient;

static bool camd_open(CamdClient* c, V4L2Capture* geom) {
    memset(c, 0, sizeof(*c));
    c->sock = -1;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, CAMD_SOCKET, sizeof(CAMD_SOCKET) - 1);
    socklen_t alen = offsetof(struct sockaddr_un, sun_path) + sizeof(CAMD_SOCKET) - 1;
    /* yolo-app-start starts yolo_camd first; allow it a few seconds */
    for (int tries = 0; tries < 20; tries++) {
        c->sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        if (c->sock >= 0 && connect(c->sock, (struct sockaddr*)&addr, alen) == 0) break;
        if (c->sock >= 0) close(c->sock);
        c->sock = -1;
        usleep(500000);
    }
    if (c->sock < 0) { fprintf(stderr, "RESULT: CAMD_CONNECT_FAIL (is yolo_camd running?)\n"); return false; }
    struct timeval tv = { 3, 0 };   /* a frame comes every ~33ms; 3s means camd is stuck */
    setsockopt(c->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    CamdHello h;
    char ctrl[CMSG_SPACE(sizeof(int) * CAMD_MAX_BUFFERS)];
    struct iovec iov = { &h, sizeof(h) };
    struct msghdr msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov; msg.msg_iovlen = 1;
    msg.msg_control = ctrl; msg.msg_controllen = sizeof(ctrl);
    ssize_t n = recvmsg(c->sock, &msg, MSG_CMSG_CLOEXEC);
    struct cmsghdr* cm = n > 0 ? CMSG_FIRSTHDR(&msg) : NULL;
    if (n != (ssize_t)sizeof(h) || h.magic != CAMD_MAGIC || !cm || cm->cmsg_type != SCM_RIGHTS ||
        h.n_buffers < 1 || h.n_buffers > CAMD_MAX_BUFFERS ||
        cm->cmsg_len != CMSG_LEN(sizeof(int) * h.n_buffers)) {
        fprintf(stderr, "RESULT: CAMD_HELLO_BAD\n");
        return false;
    }
    int fds[CAMD_MAX_BUFFERS];
    memcpy(fds, CMSG_DATA(cm), sizeof(int) * h.n_buffers);
    c->n = (int)h.n_buffers;
    for (int i = 0; i < c->n; i++) {
        c->buffers[i].fd = fds[i];
        c->buffers[i].length = h.length[i];
        c->buffers[i].start = mmap(NULL, h.length[i], PROT_READ, MAP_SHARED, fds[i], 0);
        if (c->buffers[i].start == MAP_FAILED) { c->buffers[i].start = NULL; perror("mmap camd buffer"); return false; }
    }
    if (h.stride != h.width) {   /* nv12_to_rgb assumes packed rows */
        fprintf(stderr, "RESULT: CAMD_STRIDE_UNSUPPORTED stride %u width %u\n", h.stride, h.width);
        return false;
    }
    memset(geom, 0, sizeof(*geom));
    geom->fd = -1;
    geom->width = (int)h.width;
    geom->height = (int)h.height;
    printf("RESULT: CAMD_CAPTURE_READY %ux%u NV12 buffers=%d (stream owned by yolo_camd)\n", h.width, h.height, c->n);
    return true;
}

static bool camd_capture_frame(CamdClient* c, int w, int h, unsigned char* out_rgb) {
    char cmd = 'N';
    CamdFrame f;
    if (send(c->sock, &cmd, 1, MSG_NOSIGNAL) != 1) { perror("camd send"); return false; }
    if (recv(c->sock, &f, sizeof(f), 0) != (ssize_t)sizeof(f) || f.index < 0 || f.index >= c->n) {
        perror("camd recv");
        return false;
    }
    int fd = c->buffers[f.index].fd;
    struct dma_buf_sync sync = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    nv12_to_rgb((const unsigned char*)c->buffers[f.index].start, w, h, out_rgb);
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    cmd = 'R';
    send(c->sock, &cmd, 1, MSG_NOSIGNAL);
    return true;
}

static void camd_close(CamdClient* c) {
    for (int i = 0; i < c->n; i++) {
        if (c->buffers[i].start) munmap(c->buffers[i].start, c->buffers[i].length);
        close(c->buffers[i].fd);
    }
    if (c->sock >= 0) close(c->sock);
}

/* --- RGA colour conversion (default with yolo_camd, 2026-10-12) ---
 * The RV1106's 2D engine does the camera NV12 -> RGB conversion (1.1 ms vs
 * 22 ms on the CPU), the NV12 -> model-size RGB resize in the same pass
 * (replacing the CPU downscale), and RGB -> NV12 for the hardware JPEG
 * encoder. Measured with scripts/rga_bench.c.
 *
 * Every buffer the RGA touches is a DMA-BUF (camd's capture buffers, CMA heap
 * allocations here, the encoder's MB block), imported by fd -- NEVER a plain
 * virtual address. This RGA core has no IOMMU; a virtual-address job is
 * rejected and the driver's error path corrupted the process's memory
 * accounting, so the process's exit oopsed the kernel (panic + reboot,
 * 2026-10-12, build/rga-crash-20261012).
 *
 * Cache rules for buffers both the CPU and the RGA use: flush (SYNC_END)
 * before the RGA writes or reads one, invalidate (SYNC_START) after the RGA
 * writes one. Any RGA failure switches to the CPU path for good.
 * LUCKFOX_RGA=0: CPU path. */
struct rk_dma_heap_allocation_data { uint64_t len; uint32_t fd; uint32_t fd_flags; uint64_t heap_flags; };
#define RK_DMA_HEAP_IOCTL_ALLOC _IOWR('R', 0x0, struct rk_dma_heap_allocation_data)
typedef struct { int fd; size_t len; unsigned char* va; rga_buffer_handle_t h; } RgaBuf;
static bool g_rga = false;
static RgaBuf g_rga_frames[3], g_rga_small;
static rga_buffer_handle_t g_rga_cam[CAMD_MAX_BUFFERS];
static int g_rga_ncam = 0;

static void dmabuf_sync(int fd, unsigned flags) {
    struct dma_buf_sync s = { flags };
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

static bool rga_alloc(int heap, RgaBuf* b, int w, int h, int fmt, size_t len) {
    memset(b, 0, sizeof(*b));
    b->fd = -1;
    struct rk_dma_heap_allocation_data a = { len, 0, O_RDWR | O_CLOEXEC, 0 };
    if (ioctl(heap, RK_DMA_HEAP_IOCTL_ALLOC, &a) < 0) { perror("rk dma heap alloc"); return false; }
    b->fd = (int)a.fd;
    b->len = len;
    b->va = (unsigned char*)mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, b->fd, 0);
    if (b->va == MAP_FAILED) { b->va = NULL; perror("mmap dma-buf"); return false; }
    im_handle_param_t p = { (uint32_t)w, (uint32_t)h, (uint32_t)fmt };
    b->h = importbuffer_fd(b->fd, &p);
    return b->h != 0;
}

static RgaBuf* rga_frame_for(const unsigned char* p) {
    for (int i = 0; i < 3; i++) if (g_rga_frames[i].va == p) return &g_rga_frames[i];
    return NULL;
}

/* Allocates the 3 full-size RGB frames and the model-size frame as DMA-BUFs
 * and imports camd's capture buffers. On success frames[] / *frame_small
 * point at the DMA-BUF mappings. */
static bool rga_init(CamdClient* camd, int w, int h, int mw, int mh, unsigned char* frames[3], unsigned char** frame_small) {
    const char* opt = getenv("LUCKFOX_RGA");
    if (opt && opt[0] == '0') { printf("RESULT: RGA off (LUCKFOX_RGA=0)\n"); return false; }
    int heap = open("/dev/rk_dma_heap/rk-dma-heap-cma", O_RDWR | O_CLOEXEC);
    if (heap < 0) { perror("open rk-dma-heap-cma"); return false; }
    bool ok = true;
    for (int i = 0; i < 3 && ok; i++) ok = rga_alloc(heap, &g_rga_frames[i], w, h, RK_FORMAT_RGB_888, (size_t)w * h * 3);
    if (ok) ok = rga_alloc(heap, &g_rga_small, mw, mh, RK_FORMAT_RGB_888, (size_t)mw * mh * 3);
    close(heap);
    for (int i = 0; i < camd->n && ok; i++) {
        im_handle_param_t p = { (uint32_t)w, (uint32_t)h, (uint32_t)RK_FORMAT_YCbCr_420_SP };
        g_rga_cam[i] = importbuffer_fd(camd->buffers[i].fd, &p);
        ok = g_rga_cam[i] != 0;
        g_rga_ncam = i + 1;
    }
    if (!ok) { printf("RESULT: RGA setup failed, CPU colour conversion\n"); return false; }
    for (int i = 0; i < 3; i++) frames[i] = g_rga_frames[i].va;
    *frame_small = g_rga_small.va;
    g_rga = true;
    printf("RESULT: RGA colour conversion on (NV12->RGB %dx%d, NV12->RGB %dx%d, RGB->NV12 for VENC)\n", w, h, mw, mh);
    return true;
}

static void rga_shutdown(void) {
    for (int i = 0; i < g_rga_ncam; i++) if (g_rga_cam[i]) releasebuffer_handle(g_rga_cam[i]);
    g_rga_ncam = 0;
    RgaBuf* all[4] = { &g_rga_frames[0], &g_rga_frames[1], &g_rga_frames[2], &g_rga_small };
    for (int i = 0; i < 4; i++) {
        if (all[i]->h) releasebuffer_handle(all[i]->h);
        if (all[i]->va) munmap(all[i]->va, all[i]->len);
        if (all[i]->fd >= 0) close(all[i]->fd);
        memset(all[i], 0, sizeof(RgaBuf));
        all[i]->fd = -1;
    }
    g_rga = false;
}

/* Next camd frame -> full-size RGB in g_rga_frames[fb] and model-size RGB
 * in g_rga_small, both on the RGA. Returns false on any failure; if the RGA
 * itself failed, g_rga is cleared (caller falls back to the CPU path). */
static bool camd_capture_frame_rga(CamdClient* c, int fb, int w, int h, int mw, int mh, bool want_small,
                                   const std::function<void()>* while_waiting = NULL) {
    char cmd = 'N';
    CamdFrame f;
    if (send(c->sock, &cmd, 1, MSG_NOSIGNAL) != 1) { perror("camd send"); return false; }
    if (while_waiting) {
        /* up to ~33 ms until the camera's next frame: keep checking (e.g. so a
         * finished NPU job is followed by the next one at once) */
        struct pollfd pw = { c->sock, POLLIN, 0 };
        for (int waited = 0; waited < 3000 && poll(&pw, 1, 2) == 0; waited += 2) (*while_waiting)();
    }
    if (recv(c->sock, &f, sizeof(f), 0) != (ssize_t)sizeof(f) || f.index < 0 || f.index >= c->n || f.index >= g_rga_ncam) {
        perror("camd recv");
        return false;
    }
    RgaBuf* full = &g_rga_frames[fb];
    dmabuf_sync(full->fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
    dmabuf_sync(g_rga_small.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
    rga_buffer_t src = wrapbuffer_handle(g_rga_cam[f.index], w, h, RK_FORMAT_YCbCr_420_SP);
    IM_STATUS st = imcvtcolor(src, wrapbuffer_handle(full->h, w, h, RK_FORMAT_RGB_888),
                              RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT);
    if (want_small && (st == IM_STATUS_SUCCESS || st == IM_STATUS_NOERROR)) {
        im_rect srect = { 0, 0, w, h }, drect = { 0, 0, mw, mh }, prect = { 0, 0, 0, 0 };
        rga_buffer_t none;
        memset(&none, 0, sizeof(none));
        st = improcess(src, wrapbuffer_handle(g_rga_small.h, mw, mh, RK_FORMAT_RGB_888), none,
                       srect, drect, prect, IM_SYNC | IM_YUV_TO_RGB_BT601_LIMIT);
    }
    cmd = 'R';
    send(c->sock, &cmd, 1, MSG_NOSIGNAL);
    if (st != IM_STATUS_SUCCESS && st != IM_STATUS_NOERROR) {
        printf("RESULT: RGA_FAILED (%s), CPU colour conversion from now on\n", imStrError(st));
        g_rga = false;
        return false;
    }
    dmabuf_sync(full->fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW);
    if (want_small) dmabuf_sync(g_rga_small.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    return true;
}

#ifdef LUCKFOX_ROCKIT_VI
/* Rockit VI + in-process 3A, mirroring media/samples/example/vi/sample_vi.c
 * and common/{sample_comm_vi.c, isp3.x/sample_comm_isp.c}: 3A init+start
 * first, then RK_MPI_SYS_Init and the VI device/pipe/channel; teardown in
 * reverse (channel, device, SYS_Exit, then 3A stop+deinit last). */
#define ROCKIT_CAM_ID 0
#define ROCKIT_CHN_ID 0
#define ROCKIT_IQ_DIR "/oem/usr/share/iqfiles"
typedef struct {
    rk_aiq_sys_ctx_t* aiq;
    bool sys_init, dev_enabled, chn_enabled;
    int width, height;
    unsigned char* compact; /* NV12 without row padding, if the buffer has any */
} RockitCam;

static bool rockit_cam_open(RockitCam* c, int w, int h) {
    memset(c, 0, sizeof(*c));
    c->width = w; c->height = h;
    setenv("HDR_MODE", "0", 1); /* normal (non-HDR), set before init as the sample does */
    rk_aiq_static_info_t info;
    memset(&info, 0, sizeof(info));
    if (rk_aiq_uapi2_sysctl_enumStaticMetasByPhyId(ROCKIT_CAM_ID, &info) != XCAM_RETURN_NO_ERROR) {
        printf("RESULT: AIQ_ENUM_FAIL\n"); return false;
    }
    printf("RESULT: AIQ_SENSOR %s\n", info.sensor_info.sensor_name);
    rk_aiq_uapi2_sysctl_preInit_devBufCnt(info.sensor_info.sensor_name, "rkraw_rx", 2);
    c->aiq = rk_aiq_uapi2_sysctl_init(info.sensor_info.sensor_name, ROCKIT_IQ_DIR, NULL, NULL);
    if (!c->aiq) { printf("RESULT: AIQ_INIT_FAIL\n"); return false; }
    if (rk_aiq_uapi2_sysctl_prepare(c->aiq, 0, 0, RK_AIQ_WORKING_MODE_NORMAL) != XCAM_RETURN_NO_ERROR) {
        printf("RESULT: AIQ_PREPARE_FAIL\n"); return false;
    }
    if (rk_aiq_uapi2_sysctl_start(c->aiq) != XCAM_RETURN_NO_ERROR) { printf("RESULT: AIQ_START_FAIL\n"); return false; }

    if (RK_MPI_SYS_Init() != RK_SUCCESS) { printf("RESULT: ROCKIT_SYS_INIT_FAIL\n"); return false; }
    c->sys_init = true;

    VI_DEV_ATTR_S dev_attr;
    memset(&dev_attr, 0, sizeof(dev_attr));
    RK_S32 r = RK_MPI_VI_GetDevAttr(ROCKIT_CAM_ID, &dev_attr);
    if (r == RK_ERR_VI_NOT_CONFIG && RK_MPI_VI_SetDevAttr(ROCKIT_CAM_ID, &dev_attr) != RK_SUCCESS) {
        printf("RESULT: VI_SET_DEV_ATTR_FAIL\n"); return false;
    }
    if (RK_MPI_VI_GetDevIsEnable(ROCKIT_CAM_ID) != RK_SUCCESS) {
        if (RK_MPI_VI_EnableDev(ROCKIT_CAM_ID) != RK_SUCCESS) { printf("RESULT: VI_ENABLE_DEV_FAIL\n"); return false; }
        c->dev_enabled = true;
        VI_DEV_BIND_PIPE_S bind;
        memset(&bind, 0, sizeof(bind));
        bind.u32Num = 1;
        bind.PipeId[0] = ROCKIT_CAM_ID;
        if (RK_MPI_VI_SetDevBindPipe(ROCKIT_CAM_ID, &bind) != RK_SUCCESS) { printf("RESULT: VI_BIND_PIPE_FAIL\n"); return false; }
    }
    VI_CHN_ATTR_S chn;
    memset(&chn, 0, sizeof(chn));
    chn.stIspOpt.u32BufCount = 2;
    chn.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    chn.u32Depth = 1; /* frames are fetched with GetChnFrame (no bound consumer) */
    chn.enPixelFormat = RK_FMT_YUV420SP;
    chn.enCompressMode = COMPRESS_MODE_NONE;
    chn.stFrameRate.s32SrcFrameRate = -1;
    chn.stFrameRate.s32DstFrameRate = -1;
    chn.stSize.u32Width = w;
    chn.stSize.u32Height = h;
    if (RK_MPI_VI_SetChnAttr(ROCKIT_CAM_ID, ROCKIT_CHN_ID, &chn) != RK_SUCCESS) { printf("RESULT: VI_SET_CHN_ATTR_FAIL\n"); return false; }
    if (RK_MPI_VI_EnableChn(ROCKIT_CAM_ID, ROCKIT_CHN_ID) != RK_SUCCESS) { printf("RESULT: VI_ENABLE_CHN_FAIL\n"); return false; }
    c->chn_enabled = true;
    printf("RESULT: ROCKIT_VI_READY %dx%d NV12 (in-process 3A, iq %s)\n", w, h, ROCKIT_IQ_DIR);
    return true;
}

static bool rockit_cam_capture(RockitCam* c, unsigned char* out_rgb) {
    VIDEO_FRAME_INFO_S f;
    memset(&f, 0, sizeof(f));
    if (RK_MPI_VI_GetChnFrame(ROCKIT_CAM_ID, ROCKIT_CHN_ID, &f, 1000) != RK_SUCCESS) {
        printf("RESULT: VI_GET_FRAME_TIMEOUT\n");
        return false;
    }
    RK_MPI_SYS_MmzFlushCache(f.stVFrame.pMbBlk, RK_TRUE); /* invalidate before the CPU reads the DMA buffer */
    const unsigned char* nv12 = (const unsigned char*)RK_MPI_MB_Handle2VirAddr(f.stVFrame.pMbBlk);
    int vw = f.stVFrame.u32VirWidth ? (int)f.stVFrame.u32VirWidth : c->width;
    int vh = f.stVFrame.u32VirHeight ? (int)f.stVFrame.u32VirHeight : c->height;
    if (vw == c->width && vh == c->height) {
        nv12_to_rgb(nv12, c->width, c->height, out_rgb);
    } else {
        /* padded rows/planes: compact into a plain NV12 copy first */
        if (!c->compact) c->compact = (unsigned char*)malloc((size_t)c->width * c->height * 3 / 2);
        for (int y = 0; y < c->height; y++) memcpy(c->compact + (size_t)y * c->width, nv12 + (size_t)y * vw, c->width);
        const unsigned char* uv = nv12 + (size_t)vw * vh;
        unsigned char* uv_out = c->compact + (size_t)c->width * c->height;
        for (int y = 0; y < c->height / 2; y++) memcpy(uv_out + (size_t)y * c->width, uv + (size_t)y * vw, c->width);
        nv12_to_rgb(c->compact, c->width, c->height, out_rgb);
    }
    RK_MPI_VI_ReleaseChnFrame(ROCKIT_CAM_ID, ROCKIT_CHN_ID, &f);
    return true;
}

static void rockit_cam_close(RockitCam* c) {
    if (c->chn_enabled) RK_MPI_VI_DisableChn(ROCKIT_CAM_ID, ROCKIT_CHN_ID);
    if (c->dev_enabled) RK_MPI_VI_DisableDev(ROCKIT_CAM_ID);
    if (c->sys_init) RK_MPI_SYS_Exit();
    if (c->aiq) {
        rk_aiq_uapi2_sysctl_stop(c->aiq, false);
        rk_aiq_uapi2_sysctl_deinit(c->aiq);
    }
    free(c->compact);
    memset(c, 0, sizeof(*c));
    printf("RESULT: ROCKIT_VI_CLOSED\n");
}
#endif

/* Start time of the NPU job in flight (0 = none). A job that never finishes
 * means the NPU hung -- seen 2026-10-11: the worker stuck uninterruptibly in
 * rknpu_submit_ioctl while the main thread kept running, so a main-thread
 * heartbeat alone hid the hang. heartbeat() skips beats while a job is >10s overdue. */
/* monotonic seconds (the wall clock can jump when the browser sets it) */
static time_t mono_now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec;
}
static volatile time_t g_npu_job_started = 0;

/* --- NPU worker thread (2026-10-11 optimisation) ---
 * Measured per 960x540 frame before this: CPU ~67ms (capture+convert 16,
 * downscale 9, quantize 6, HUD 4, JPEG 27, misc) and NPU 61ms (rknn_run,
 * during which the single CPU core just waited) -- run back to back, 7.8fps.
 * Now the NPU works on frame N in this thread while the main thread
 * captures/prepares N+1 and draws/encodes N-1 with N-1's own detections, so
 * a frame costs ~max(CPU, NPU) instead of the sum. Only this thread touches
 * the RKNN context after setup. Two input tensors are alternated so the CPU
 * can fill one while the NPU reads the other; outputs are decoded here,
 * right after the run, before the next run can overwrite them. */
typedef struct {
    rknn_context ctx;
    rknn_tensor_mem* input_mems[3];   /* 0/1: pipelined loop; 0-2: decoupled loop */
    rknn_tensor_attr input_attr;
    std::vector<rknn_tensor_attr>* output_attrs;
    std::vector<rknn_tensor_mem*>* output_mems;

    pthread_mutex_t lock;
    pthread_cond_t cond;
    int job_idx;       /* -1 = no job queued */
    int next_idx;      /* staged job the worker starts by itself when job_idx finishes; -1 = none */
    bool job_done;
    bool quit;
    unsigned done_seq; /* jobs finished so far; last_done = slot of the newest */
    int last_done;
    /* results, per input slot */
    std::vector<Detection> dets[3];
    double npu_ms[3], post_ms[3], copy_ms[3], post_cpu_ms[3];
    int ret[3];
} NpuWorker;

static void* npu_worker_main(void* arg) {
    NpuWorker* w = (NpuWorker*)arg;
    for (;;) {
        pthread_mutex_lock(&w->lock);
        while (w->job_idx < 0 && !w->quit) pthread_cond_wait(&w->cond, &w->lock);
        if (w->quit) { pthread_mutex_unlock(&w->lock); break; }
        int idx = w->job_idx;
        g_npu_job_started = mono_now_s();   /* per job, for heartbeat()'s NPU-hang check */
        pthread_mutex_unlock(&w->lock);

        struct timespec t0, t1, t2;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        w->copy_ms[idx] = 0;
        int ret = rknn_set_io_mem(w->ctx, w->input_mems[idx], &w->input_attr);
        if (ret == RKNN_SUCC) ret = rknn_run(w->ctx, NULL);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        struct timespec c1, c2;   /* this thread's own CPU time, vs wall time */
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        std::vector<Detection> all_dets;
        if (ret == RKNN_SUCC) {
            std::vector<rknn_tensor_attr>& oa = *w->output_attrs;
            std::vector<rknn_tensor_mem*>& om = *w->output_mems;
            for (int s = 0; s < NUM_SCALES; s++) {
                rknn_tensor_attr& box_attr = oa[s * 3 + 0];
                rknn_tensor_attr& score_attr = oa[s * 3 + 1];
                rknn_tensor_attr& sum_attr = oa[s * 3 + 2];
                int grid_h = box_attr.dims[1], grid_w = box_attr.dims[2], dfl_len = box_attr.dims[3] / 4;
                /* The class-score tensor is copied into cached RAM in one bulk
                 * memcpy before the decode scans it: the NPU output buffers
                 * from rknn_create_mem are read slowly by the CPU byte by
                 * byte, which cost ~20 ms per frame once the score-sum
                 * pre-filter stopped rejecting cells (800x448 model). */
                static std::vector<int8_t> score_copy[3];
                score_copy[s].resize(score_attr.size);
                struct timespec tc0, tc1;
                clock_gettime(CLOCK_MONOTONIC, &tc0);
                memcpy(score_copy[s].data(), om[s * 3 + 1]->virt_addr, score_attr.size);
                clock_gettime(CLOCK_MONOTONIC, &tc1);
                w->copy_ms[idx] += (tc1.tv_sec - tc0.tv_sec) * 1000.0 + (tc1.tv_nsec - tc0.tv_nsec) / 1e6;
                decode_scale((int8_t*)om[s * 3 + 0]->virt_addr, box_attr.zp, box_attr.scale,
                             score_copy[s].data(), score_attr.zp, score_attr.scale,
                             (int8_t*)om[s * 3 + 2]->virt_addr, sum_attr.zp, sum_attr.scale,
                             grid_h, grid_w, STRIDES[s], dfl_len, all_dets);
            }
        }
        /* LUCKFOX_DUMP_OUTPUTS=1: write the raw output buffers (size_with_stride
         * bytes each) of the 30th inference to /tmp once, to check the real
         * memory layout of a model's outputs. */
        static int dump_countdown = getenv("LUCKFOX_DUMP_OUTPUTS") ? 30 : -1;
        if (ret == RKNN_SUCC && dump_countdown > 0 && --dump_countdown == 0) {
            for (size_t k = 0; k < w->output_attrs->size(); k++) {
                char path[64];
                snprintf(path, sizeof(path), "/tmp/rknn-out%zu.bin", k);
                FILE* fp = fopen(path, "wb");
                if (fp) {
                    fwrite((*w->output_mems)[k]->virt_addr, 1, (*w->output_attrs)[k].size_with_stride, fp);
                    fclose(fp);
                }
            }
            printf("RESULT: OUTPUTS_DUMPED /tmp/rknn-out*.bin\n");
        }
        std::vector<Detection> dets = nms(all_dets);
        clock_gettime(CLOCK_MONOTONIC, &t2);

        pthread_mutex_lock(&w->lock);
        w->dets[idx].swap(dets);
        w->ret[idx] = ret;
        w->npu_ms[idx] = (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6;
        w->post_ms[idx] = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_nsec - t1.tv_nsec) / 1e6;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c2);
        w->post_cpu_ms[idx] = (c2.tv_sec - c1.tv_sec) * 1000.0 + (c2.tv_nsec - c1.tv_nsec) / 1e6;
        w->done_seq++;
        w->last_done = idx;
        if (w->next_idx >= 0) {
            /* a frame is staged: start it now, without waiting for the main
             * thread to notice (decoupled display) */
            w->job_idx = w->next_idx;
            w->next_idx = -1;
        } else {
            w->job_idx = -1;
            w->job_done = true;
            g_npu_job_started = 0;
        }
        pthread_cond_broadcast(&w->cond);
        pthread_mutex_unlock(&w->lock);
    }
    return NULL;
}

static void npu_submit(NpuWorker* w, int idx) {
    g_npu_job_started = mono_now_s();
    pthread_mutex_lock(&w->lock);
    w->job_idx = idx;
    w->job_done = false;
    pthread_cond_broadcast(&w->cond);
    pthread_mutex_unlock(&w->lock);
}

/* Decoupled loop: run slot idx now if the NPU is idle, else make it the job
 * the worker starts next (replacing any older staged frame). */
static void npu_stage(NpuWorker* w, int idx) {
    pthread_mutex_lock(&w->lock);
    if (w->job_idx < 0) {
        w->job_idx = idx;
        w->job_done = false;
        g_npu_job_started = mono_now_s();
        pthread_cond_broadcast(&w->cond);
    } else {
        w->next_idx = idx;
    }
    pthread_mutex_unlock(&w->lock);
}

/* Waits until the worker is idle with nothing staged. */
static void npu_drain(NpuWorker* w) {
    pthread_mutex_lock(&w->lock);
    w->next_idx = -1;
    while (w->job_idx >= 0) pthread_cond_wait(&w->cond, &w->lock);
    w->job_done = true;
    pthread_mutex_unlock(&w->lock);
    g_npu_job_started = 0;
}

static bool npu_is_done(NpuWorker* w) {
    pthread_mutex_lock(&w->lock);
    bool done = w->job_done;
    pthread_mutex_unlock(&w->lock);
    return done;
}

static void npu_wait(NpuWorker* w) {
    pthread_mutex_lock(&w->lock);
    while (!w->job_done) pthread_cond_wait(&w->cond, &w->lock);
    pthread_mutex_unlock(&w->lock);
    g_npu_job_started = 0;
}

/* --- per-session object log (2026-10-10, web UI) ---
 * A "session" is one stream viewer's connection: it starts on /stream and
 * the log is wiped when that viewer disconnects or is taken over. Raw
 * per-frame detections would list the same car ~8 times a second, so
 * detections are matched frame-to-frame (same class + IoU, greedy) into
 * objects: each real object appears once, with first/last seen and best
 * confidence. Deliberately simple -- no motion model, so a fast object or
 * a long occlusion can be counted twice. Lives only in RAM. */
#define SESSION_MAX_OBJECTS 200   /* ring buffer; oldest entries roll off */
#define TRACK_MATCH_IOU 0.3f
#define TRACK_MAX_MISSED 8        /* ~1s at 8fps before an object counts as gone */

typedef struct {
    int id, cls_id, missed;
    float best_score;
    double first_s, last_s;
    Detection box;
    bool active;
} SessionObject;

static struct {
    int session_id;
    struct timespec t0;
    int total; /* objects ever created this session; ring index = (id-1) % MAX */
    SessionObject objs[SESSION_MAX_OBJECTS];
    int class_counts[OBJ_CLASS_NUM];
} g_session;

static double session_elapsed_s(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - g_session.t0.tv_sec) + (now.tv_nsec - g_session.t0.tv_nsec) / 1e9;
}

static void session_reset(void) {
    g_session.session_id++;
    g_session.total = 0;
    memset(g_session.objs, 0, sizeof(g_session.objs));
    memset(g_session.class_counts, 0, sizeof(g_session.class_counts));
    clock_gettime(CLOCK_MONOTONIC, &g_session.t0);
}

static void session_update(const std::vector<Detection>& dets, std::vector<int>* ids_out) {
    double t = session_elapsed_s();
    if (ids_out) ids_out->clear();
    int n_live = std::min(g_session.total, SESSION_MAX_OBJECTS);
    bool matched[SESSION_MAX_OBJECTS] = {false};
    for (const Detection& d : dets) {
        int best = -1;
        float best_iou = TRACK_MATCH_IOU;
        for (int i = 0; i < n_live; i++) {
            SessionObject& o = g_session.objs[i];
            if (!o.active || matched[i] || o.cls_id != d.cls_id) continue;
            float v = iou(o.box, d);
            if (v > best_iou) { best_iou = v; best = i; }
        }
        if (best < 0) {
            int id = ++g_session.total;
            best = (id - 1) % SESSION_MAX_OBJECTS;
            SessionObject& o = g_session.objs[best];
            o.id = id;
            o.cls_id = d.cls_id;
            o.best_score = 0;
            o.first_s = t;
            o.active = true;
            g_session.class_counts[d.cls_id]++;
            n_live = std::min(g_session.total, SESSION_MAX_OBJECTS);
        }
        SessionObject& o = g_session.objs[best];
        matched[best] = true;
        if (ids_out) ids_out->push_back(o.id);
        o.box = d;
        o.last_s = t;
        o.missed = 0;
        if (d.score > o.best_score) o.best_score = d.score;
    }
    for (int i = 0; i < n_live; i++) {
        SessionObject& o = g_session.objs[i];
        if (o.active && !matched[i] && ++o.missed > TRACK_MAX_MISSED) o.active = false;
    }
}

/* View mode, switchable live from the UI (/api/view?mode=hud|plain):
 * full apex_hud overlay, or plain video with just the YOLO boxes. */
static bool g_view_hud = true;

static void draw_plain_overlay(image_buffer_t* img, const std::vector<Detection>& dets,
                               float scale_x, float scale_y) {
    int fsz = apex_hud_font_size(img, 7);
    for (const Detection& d : dets) {
        int x = (int)(d.x * scale_x), y = (int)(d.y * scale_y);
        int w = (int)(d.w * scale_x), h = (int)(d.h * scale_y);
        draw_rectangle(img, x, y, w, h, COLOR_GREEN, 2);
        char label[96];
        snprintf(label, sizeof(label), "%s %.0f%%", class_names[d.cls_id], d.score * 100);
        draw_text(img, label, x, std::max(y - fsz - 4, 0), COLOR_GREEN, fsz);
    }
}

/* --- navigation output (2026-10-11) ---
 * Per-frame detections for other systems (e.g. the Pi 5's navigation):
 *   /api/detections         latest frame as JSON
 *   /api/detections/stream  Server-Sent Events, one JSON event per frame
 * Coordinates are normalised to the frame (0..1, origin top-left); cx/cy is
 * the bounding-box centre (YOLO gives boxes, so this approximates the
 * object's centre of mass). "id" is the frame-to-frame track id from the
 * session tracker. bearing/elevation (degrees, +right/+up from the optical
 * axis) are included when LUCKFOX_CAM_HFOV_DEG / LUCKFOX_CAM_VFOV_DEG are set
 * (pinhole model). One stream subscriber at a time; a newer one replaces it.
 * A subscriber keeps the detector running with no video viewer, and then
 * the HUD drawing and JPEG encode are skipped entirely. */
static int g_nav_fd = -1;
static std::string g_last_det_json = "{\"active\":false}";
static float g_hfov_deg = 0, g_vfov_deg = 0;

static std::string build_det_json(int frame, const std::vector<Detection>& dets, const std::vector<int>& ids,
                                  int display_w, int display_h, int has_tilt, float pitch, float roll) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    char tmp[320];
    snprintf(tmp, sizeof(tmp),
             "{\"active\":true,\"frame\":%d,\"t_ms\":%lld,\"w\":%d,\"h\":%d,\"session\":%d,"
             "\"has_tilt\":%s,\"pitch\":%.1f,\"roll\":%.1f,\"dets\":[",
             frame, (long long)now.tv_sec * 1000 + now.tv_nsec / 1000000, display_w, display_h,
             g_session.session_id, has_tilt ? "true" : "false", pitch, roll);
    std::string j = tmp;
    for (size_t i = 0; i < dets.size(); i++) {
        const Detection& d = dets[i];
        float bx = d.x / g_model_w, by = d.y / g_model_h, bw = d.w / g_model_w, bh = d.h / g_model_h;
        float cx = bx + bw / 2, cy = by + bh / 2;
        snprintf(tmp, sizeof(tmp),
                 "%s{\"id\":%d,\"cls\":\"%s\",\"score\":%.2f,\"cx\":%.4f,\"cy\":%.4f,"
                 "\"bx\":%.4f,\"by\":%.4f,\"bw\":%.4f,\"bh\":%.4f",
                 i ? "," : "", i < ids.size() ? ids[i] : 0, class_names[d.cls_id], d.score, cx, cy, bx, by, bw, bh);
        j += tmp;
        if (g_hfov_deg > 0) {
            float b = atanf((cx - 0.5f) * 2.0f * tanf(g_hfov_deg * (float)M_PI / 360.0f)) * 180.0f / (float)M_PI;
            snprintf(tmp, sizeof(tmp), ",\"bearing_deg\":%.2f", b);
            j += tmp;
        }
        if (g_vfov_deg > 0) {
            float e = atanf((0.5f - cy) * 2.0f * tanf(g_vfov_deg * (float)M_PI / 360.0f)) * 180.0f / (float)M_PI;
            snprintf(tmp, sizeof(tmp), ",\"elev_deg\":%.2f", e);
            j += tmp;
        }
        j += "}";
    }
    j += "]}";
    return j;
}

/* --- minimal single-viewer HTTP server: web UI + MJPEG stream ---
 *
 * One process, one port, no threads (2026-10-10, plug-and-play web UI):
 *   /             -> static files from the web root (index.html = the UI)
 *   /stream       -> the MJPEG HUD stream (the single viewer slot)
 *   /api/status   -> small JSON of live pipeline stats for the UI
 * Single viewer by design (README "Connectivity", 2026-09-20): a new
 * /stream request REPLACES the current viewer instead of queueing behind
 * it -- a browser refresh or a second laptop just takes over. Pages and
 * /api requests are answered between frames (poll() with zero timeout,
 * so they never stall the pipeline) and immediately while idle. With no
 * viewer the pipeline doesn't run at all: output only on connection.
 *
 * Web root: $LUCKFOX_WEB_ROOT, default "www" relative to the working
 * directory (/home/pico/rknn_test/www under the systemd unit). If
 * index.html is missing, a tiny built-in page with just the stream is
 * served instead, so the stream is never unreachable. */
static int server_fd = -1;
static int client_fd = -1;
/* Set by SIGTERM/SIGINT: the main loops finish the current frame and exit
 * through the normal cleanup path. Real reason (2026-10-10): `killall` on a
 * process in the middle of a Rockit VENC call deadlocked the encoder driver
 * and froze the whole board; the hardware encoder must be shut down cleanly. */
static volatile sig_atomic_t g_quit = 0;

/* Heartbeat for /usr/bin/yolo-health (2026-10-11): touched about once a
 * second while running, idle or streaming. The board's freezes leave the
 * kernel half-alive (USB networking dead, watchdog feeder still running),
 * with this process stuck; a stale heartbeat makes the health monitor stop
 * the watchdog feeder so the hardware watchdog resets the board. */
/* Start-up / first-frame step markers written into the kernel log, so they
 * line up with kernel messages (and survive in ramoops on a supervisor
 * reset). Added 2026-10-11 after crash logs showed the eMMC failing ~5.75s
 * after the camera stream starts on a server restart -- to see exactly which
 * start-up step coincides with it instead of guessing. */
static void step_mark(const char* step) {
    static int fd = -2;
    if (fd == -2) fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
    if (fd < 0) return;
    char line[160];
    int n = snprintf(line, sizeof(line), "<6>yolo-server: %s\n", step);
    if (write(fd, line, n) < 0) { /* best effort */ }
}

#define HEARTBEAT_FILE "/tmp/yolo-heartbeat"
/* Hardware watchdog, fed by this process itself (2026-10-11). A separate
 * feeder + monitor failed in a real freeze: the kernel was half-alive, the
 * feeder kept feeding, and the monitor hung in pidof (reading /proc/<pid>/
 * cmdline of a task stuck in the kernel blocks). Now the kick comes from the
 * same place as the heartbeat -- each frame and each idle second, never
 * while an NPU job is overdue -- so if this process gets stuck anywhere the
 * board resets within WATCHDOG_TIMEOUT_S. It's disarmed only at the very
 * end of a clean exit, after all teardown (several freezes were in
 * teardown). A crash (no disarm) also resets the board. */
#define WATCHDOG_TIMEOUT_S 20
static int g_wdt_fd = -1;
static void watchdog_arm(void) {
    if (getenv("LUCKFOX_NO_WATCHDOG")) { printf("RESULT: WATCHDOG_OFF (LUCKFOX_NO_WATCHDOG set)\n"); return; }
    g_wdt_fd = open("/dev/watchdog", O_WRONLY | O_CLOEXEC);
    if (g_wdt_fd < 0) { printf("RESULT: WATCHDOG_UNAVAILABLE errno=%d\n", errno); return; }
    int t = WATCHDOG_TIMEOUT_S;
    ioctl(g_wdt_fd, WDIOC_SETTIMEOUT, &t);
    printf("RESULT: WATCHDOG_ARMED timeout=%ds\n", t);
}
static void watchdog_disarm(void) {
    if (g_wdt_fd < 0) return;
    if (write(g_wdt_fd, "V", 1) < 0) { /* best effort */ }
    close(g_wdt_fd);
    g_wdt_fd = -1;
}

static void heartbeat(void) {
    static time_t last = 0;
    time_t now = time(NULL);
    if (now == last) return;
    time_t job = g_npu_job_started;
    if (job && mono_now_s() - job > 10) return;   /* NPU job stuck: stop kicking */
    if (g_wdt_fd >= 0 && write(g_wdt_fd, "k", 1) < 0) { /* best effort */ }
    last = now;
    int fd = open(HEARTBEAT_FILE, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd >= 0) {
        char b[24];
        int n = snprintf(b, sizeof(b), "%ld\n", (long)now);
        if (write(fd, b, n) < 0) { /* best effort */ }
        close(fd);
    }
}
static void on_quit_signal(int) { g_quit = 1; }
static const char* g_web_root = "www";

static struct {
    int streaming;
    int frame;
    double fps, infer_ms, jpeg_ms, read_ms, det_hz;
    int dets;
    float cpu_pct;
    int display_w, display_h;
    const char* source;
    char viewer_ip[INET_ADDRSTRLEN];
    char viewer_id[24];
    int viewers_served;
    int has_tilt;
    float pitch, roll;
} g_status;

static int send_all(int fd, const void* data, size_t len) {
    const char* p = (const char*)data;
    while (len > 0) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1; /* includes the SO_SNDTIMEO timeout: a stalled viewer counts as gone */
        }
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static void set_socket_timeouts(int fd, int seconds) {
    struct timeval tv;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void http_send_simple(int fd, const char* status, const char* ctype, const char* body, size_t len) {
    char header[256];
    int hlen = snprintf(header, sizeof(header),
                        "HTTP/1.0 %s\r\nConnection: close\r\nCache-Control: no-cache\r\n"
                        "Content-Type: %s\r\nContent-Length: %zu\r\n\r\n", status, ctype, len);
    if (send_all(fd, header, hlen) == 0) send_all(fd, body, len);
}

static const char* content_type_for(const char* path) {
    const char* dot = strrchr(path, '.');
    if (!dot) return "application/octet-stream";
    if (!strcmp(dot, ".html")) return "text/html; charset=utf-8";
    if (!strcmp(dot, ".css")) return "text/css; charset=utf-8";
    if (!strcmp(dot, ".js")) return "text/javascript; charset=utf-8";
    if (!strcmp(dot, ".json")) return "application/json";
    if (!strcmp(dot, ".svg")) return "image/svg+xml";
    if (!strcmp(dot, ".png")) return "image/png";
    if (!strcmp(dot, ".jpg") || !strcmp(dot, ".jpeg")) return "image/jpeg";
    if (!strcmp(dot, ".ico")) return "image/x-icon";
    if (!strcmp(dot, ".xml")) return "application/xml";
    if (!strcmp(dot, ".txt")) return "text/plain; charset=utf-8";
    if (!strcmp(dot, ".woff2")) return "font/woff2";
    return "application/octet-stream";
}

static const char FALLBACK_INDEX[] =
    "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"
    "<title>Luckfox</title><body style='margin:0;background:#000'>"
    "<img src='/stream' style='width:100%;height:100vh;object-fit:contain'>";

static void http_send_file(int fd, const char* url_path) {
    char path[600];
    if (strstr(url_path, "..")) { http_send_simple(fd, "403 Forbidden", "text/plain", "forbidden\n", 10); return; }
    size_t ul = strlen(url_path);
    snprintf(path, sizeof(path), "%s%s%s", g_web_root, url_path,
             (ul == 0 || url_path[ul - 1] == '/') ? "index.html" : "");
    struct stat st;
    if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
        /* /docs -> /docs/index.html */
        strncat(path, "/index.html", sizeof(path) - strlen(path) - 1);
    }
    int file = open(path, O_RDONLY);
    if (file < 0 || fstat(file, &st) != 0) {
        if (file >= 0) close(file);
        if (!strcmp(url_path, "/") || !strcmp(url_path, "/index.html")) {
            http_send_simple(fd, "200 OK", "text/html; charset=utf-8", FALLBACK_INDEX, sizeof(FALLBACK_INDEX) - 1);
        } else {
            http_send_simple(fd, "404 Not Found", "text/plain", "not found\n", 10);
        }
        return;
    }
    char header[256];
    int hlen = snprintf(header, sizeof(header),
                        "HTTP/1.0 200 OK\r\nConnection: close\r\nCache-Control: no-cache\r\n"
                        "Content-Type: %s\r\nContent-Length: %lld\r\n\r\n",
                        content_type_for(path), (long long)st.st_size);
    if (send_all(fd, header, hlen) == 0) {
        char chunk[16384];
        ssize_t n;
        while ((n = read(file, chunk, sizeof(chunk))) > 0) {
            if (send_all(fd, chunk, (size_t)n) < 0) break;
        }
    }
    close(file);
}

static int read_mem_available_mb(void) {
    FILE* f = fopen("/proc/meminfo", "r");
    if (!f) return -1;
    char line[128];
    int kb = -1;
    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "MemAvailable: %d kB", &kb) == 1) break;
    }
    fclose(f);
    return kb < 0 ? -1 : kb / 1024;
}

static float read_soc_temp_c(void) {
    FILE* f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!f) return -1;
    int milli = 0;
    int ok = fscanf(f, "%d", &milli);
    fclose(f);
    return ok == 1 ? milli / 1000.0f : -1;
}

static void http_send_status(int fd) {
    char body[1024];
    int len = snprintf(body, sizeof(body),
        "{\"streaming\":%s,\"viewer_ip\":\"%s\",\"viewer_id\":\"%s\",\"viewers_served\":%d,"
        "\"frame\":%d,\"fps\":%.2f,\"det_hz\":%.2f,\"infer_ms\":%.1f,\"jpeg_ms\":%.1f,\"read_ms\":%.1f,"
        "\"dets\":%d,\"cpu_pct\":%.0f,\"mem_avail_mb\":%d,\"soc_temp_c\":%.1f,"
        "\"display_w\":%d,\"display_h\":%d,\"model_size\":%d,\"model_w\":%d,\"model_h\":%d,\"source\":\"%s\","
        "\"view\":\"%s\",\"session\":%d,\"session_s\":%.0f,\"objects_total\":%d,"
        "\"has_tilt\":%s,\"level_set\":%s,\"pitch\":%.1f,\"roll\":%.1f,\"nav_subscriber\":%s}",
        g_status.streaming ? "true" : "false", g_status.viewer_ip, g_status.viewer_id, g_status.viewers_served,
        g_status.frame, g_status.fps, g_status.det_hz, g_status.infer_ms, g_status.jpeg_ms, g_status.read_ms,
        g_status.dets, g_status.cpu_pct, read_mem_available_mb(), read_soc_temp_c(),
        g_status.display_w, g_status.display_h, g_model_w, g_model_w, g_model_h, g_status.source ? g_status.source : "",
        g_view_hud ? "hud" : "plain", g_session.session_id,
        g_status.streaming ? session_elapsed_s() : 0.0, g_session.total,
        g_status.has_tilt ? "true" : "false", g_level_set ? "true" : "false", g_status.pitch, g_status.roll,
        g_nav_fd >= 0 ? "true" : "false");
    http_send_simple(fd, "200 OK", "application/json", body, (size_t)len);
}

/* /api/wifi[?enable=0|1] -- WiFi on/off (2026-10-11; off by default at boot,
 * see /usr/bin/yolo-wifi). Switching runs yolo-wifi in the background so the
 * frame loop never waits on it; the UI polls status. */
static void http_send_wifi(int fd, const char* query) {
    const char* action = NULL;
    if (query && strstr(query, "enable=1")) action = "on";
    else if (query && strstr(query, "enable=0")) action = "off";
    if (action) {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "/usr/bin/yolo-wifi %s > /dev/null 2>&1 &", action);
        if (system(cmd) != 0) { /* reported through status below */ }
    }
    bool up = false;
    char ip[INET_ADDRSTRLEN] = "";
    FILE* f = fopen("/sys/class/net/wlan0/flags", "r");
    if (f) {
        unsigned int flags = 0;
        if (fscanf(f, "%x", &flags) == 1) up = flags & IFF_UP;
        fclose(f);
    }
    if (up) {
        int sfd = socket(AF_INET, SOCK_DGRAM, 0);
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "wlan0");
        if (sfd >= 0 && ioctl(sfd, SIOCGIFADDR, &ifr) == 0)
            inet_ntop(AF_INET, &((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr, ip, sizeof(ip));
        if (sfd >= 0) close(sfd);
    }
    char body[160];
    int len = snprintf(body, sizeof(body), "{\"present\":%s,\"enabled\":%s,\"ip\":\"%s\",\"pending\":%s}",
                       access("/sys/class/net/wlan0", F_OK) == 0 ? "true" : "false",
                       up ? "true" : "false", ip, action ? "true" : "false");
    http_send_simple(fd, "200 OK", "application/json", body, (size_t)len);
}

/* Newest first, at most the ring's 200 entries (or ?limit=N, e.g. the main
 * page's last 20), plus per-class totals over the whole session. */
static void http_send_objects(int fd, const char* query) {
    int limit = SESSION_MAX_OBJECTS;
    const char* l = query ? strstr(query, "limit=") : NULL;
    if (l) limit = std::max(0, atoi(l + 6));
    std::string body;
    body.reserve(16384);
    char tmp[256];
    snprintf(tmp, sizeof(tmp), "{\"session\":%d,\"streaming\":%s,\"session_s\":%.1f,\"total\":%d,\"counts\":{",
             g_session.session_id, g_status.streaming ? "true" : "false",
             g_status.streaming ? session_elapsed_s() : 0.0, g_session.total);
    body += tmp;
    bool first = true;
    for (int c = 0; c < OBJ_CLASS_NUM; c++) {
        if (!g_session.class_counts[c]) continue;
        snprintf(tmp, sizeof(tmp), "%s\"%s\":%d", first ? "" : ",", class_names[c], g_session.class_counts[c]);
        body += tmp;
        first = false;
    }
    body += "},\"objects\":[";
    int n_live = std::min(std::min(g_session.total, SESSION_MAX_OBJECTS), limit);
    for (int k = 0; k < n_live; k++) {
        int id = g_session.total - k;
        const SessionObject& o = g_session.objs[(id - 1) % SESSION_MAX_OBJECTS];
        snprintf(tmp, sizeof(tmp), "%s{\"id\":%d,\"cls\":\"%s\",\"score\":%.2f,\"first_s\":%.1f,\"last_s\":%.1f,\"active\":%s}",
                 k ? "," : "", o.id, class_names[o.cls_id], o.best_score, o.first_s, o.last_s,
                 o.active ? "true" : "false");
        body += tmp;
    }
    body += "]}";
    http_send_simple(fd, "200 OK", "application/json", body.data(), body.size());
}

/* Reads one request from a freshly accepted connection and answers it.
 * Returns true if the connection became the stream viewer (kept open
 * as client_fd); otherwise it has been answered and closed. */
static bool http_handle_connection(int fd, const char* ipstr) {
    set_socket_timeouts(fd, 2); /* a slow/dead client can't stall the pipeline for long */
    char buf[2048];
    ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) { close(fd); return false; }
    buf[n] = '\0';

    char method[8] = {0}, target[512] = {0};
    if (sscanf(buf, "%7s %511s", method, target) != 2 || strcmp(method, "GET") != 0) {
        http_send_simple(fd, "405 Method Not Allowed", "text/plain", "GET only\n", 9);
        close(fd);
        return false;
    }
    char* query = strchr(target, '?');
    if (query) *query++ = '\0';

    if (!strcmp(target, "/stream")) {
        if (client_fd >= 0) {
            printf("RESULT: VIEWER_REPLACED %s -> %s\n", g_status.viewer_ip, ipstr);
            close(client_fd);
        }
        client_fd = fd;
        /* the UI tags its stream request with ?id=<random> so each tab can
         * tell whether it is the current viewer or has been taken over */
        g_status.viewer_id[0] = '\0';
        if (query && !strncmp(query, "id=", 3)) {
            size_t k = 0;
            for (const char* q = query + 3; *q && *q != '&' && k < sizeof(g_status.viewer_id) - 1; q++) {
                if ((*q >= '0' && *q <= '9') || (*q >= 'a' && *q <= 'z') || (*q >= 'A' && *q <= 'Z')) {
                    g_status.viewer_id[k++] = *q;
                }
            }
            g_status.viewer_id[k] = '\0';
        }
        snprintf(g_status.viewer_ip, sizeof(g_status.viewer_ip), "%s", ipstr);
        g_status.viewers_served++;
        session_reset();
        printf("RESULT: CLIENT_CONNECTED %s\n", ipstr);
        const char* header =
            "HTTP/1.0 200 OK\r\n"
            "Connection: close\r\n"
            "Cache-Control: no-cache\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        if (send_all(fd, header, strlen(header)) < 0) {
            close(fd);
            client_fd = -1;
            return false;
        }
        return true;
    }
    if (!strcmp(target, "/api/detections/stream")) {
        if (g_nav_fd >= 0) {
            printf("RESULT: NAV_SUBSCRIBER_REPLACED\n");
            close(g_nav_fd);
        }
        const char* header =
            "HTTP/1.0 200 OK\r\n"
            "Connection: close\r\n"
            "Cache-Control: no-cache\r\n"
            "Content-Type: text/event-stream\r\n\r\n";
        if (send_all(fd, header, strlen(header)) < 0) { close(fd); return false; }
        g_nav_fd = fd;
        printf("RESULT: NAV_SUBSCRIBER_CONNECTED %s\n", ipstr);
        return true;
    }
    if (!strcmp(target, "/api/detections")) {
        http_send_simple(fd, "200 OK", "application/json", g_last_det_json.data(), g_last_det_json.size());
        close(fd);
        return false;
    }
    if (!strcmp(target, "/api/status")) {
        http_send_status(fd);
    } else if (!strcmp(target, "/api/level")) {
        /* ?set=1: the unit is standing level now -- store the gravity
         * direction as the reference. ?clear=1: forget it. */
        const char* msg = "ok";
        bool ok = true;
        if (query && strstr(query, "set=1")) {
            if (!g_accel || !g_accel->found || !g_accel->ema_initialized) {
                ok = false; msg = "no accelerometer reading";
            } else {
                float x = g_accel->smoothed_x, y = g_accel->smoothed_y, z = g_accel->smoothed_z;
                if (!level_from_vector(x, y, z)) { ok = false; msg = "reading is not steady gravity -- keep the unit still"; }
                else if (!level_save(x, y, z)) { msg = "levelled, but could not save to /userdata"; }
                else printf("RESULT: LEVEL_SET gravity reference (%.3f, %.3f, %.3f) m/s^2\n", x, y, z);
            }
        } else if (query && strstr(query, "clear=1")) {
            level_clear();
            unlink(LEVEL_FILE);
            printf("RESULT: LEVEL_CLEARED\n");
        }
        char body[160];
        snprintf(body, sizeof(body), "{\"ok\":%s,\"level_set\":%s,\"msg\":\"%s\"}",
                 ok ? "true" : "false", g_level_set ? "true" : "false", msg);
        http_send_simple(fd, "200 OK", "application/json", body, strlen(body));
    } else if (!strcmp(target, "/api/time")) {
        /* The board has no RTC and no network time: the web UI sends the
         * browser's clock (?epoch_ms=) when it loads, and the board takes it
         * if its own is more than 2 s off. */
        const char* q = query ? strstr(query, "epoch_ms=") : NULL;
        bool set = false;
        struct timeval now_tv;
        gettimeofday(&now_tv, NULL);
        long long board_ms = (long long)now_tv.tv_sec * 1000 + now_tv.tv_usec / 1000;
        if (q) {
            long long ms = atoll(q + 9);
            if (ms > 1700000000000LL && llabs(ms - board_ms) > 2000) {
                struct timeval tv = { (time_t)(ms / 1000), (suseconds_t)((ms % 1000) * 1000) };
                set = settimeofday(&tv, NULL) == 0;
                if (set) {
                    printf("RESULT: CLOCK_SET from browser (was off by %llds)\n", (ms - board_ms) / 1000);
                    board_ms = ms;
                }
            }
        }
        char body[96];
        snprintf(body, sizeof(body), "{\"set\":%s,\"board_ms\":%lld}", set ? "true" : "false", board_ms);
        http_send_simple(fd, "200 OK", "application/json", body, strlen(body));
        http_send_wifi(fd, query);
    } else if (!strcmp(target, "/api/objects/reset")) {
        session_reset(); /* clears the object log and per-class counts; track ids restart */
        printf("RESULT: OBJECT_LOG_RESET\n");
        http_send_objects(fd, NULL);
    } else if (!strcmp(target, "/api/objects")) {
        http_send_objects(fd, query);
    } else if (!strcmp(target, "/api/view")) {
        if (query && strstr(query, "mode=plain")) g_view_hud = false;
        else if (query && strstr(query, "mode=hud")) g_view_hud = true;
        http_send_status(fd);
    } else {
        http_send_file(fd, target);
    }
    close(fd);
    return false;
}

static void http_accept_one(void) {
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    int fd = accept(server_fd, (struct sockaddr*)&client_addr, &len);
    if (fd < 0) return;
    char ipstr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, ipstr, sizeof(ipstr));
    http_handle_connection(fd, ipstr);
}

/* Between frames: answer whatever is waiting (pages, /api/status, or a
 * new /stream that takes over), never blocking. Capped per frame so a
 * burst of page-asset requests can't starve the pipeline. */
static void http_poll_requests(void) {
    for (int i = 0; i < 8; i++) {
        struct pollfd pfd;
        pfd.fd = server_fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        if (poll(&pfd, 1, 0) <= 0 || !(pfd.revents & POLLIN)) return;
        http_accept_one();
    }
}

static int http_start_server(int port) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return -1; }
    /* backlog 8, not 1: the UI page loads a few files at once (page, then
     * /stream and /api/status); there is still only ever one stream viewer */
    listen(server_fd, 8);
    printf("RESULT: HTTP_SERVER_LISTENING port=%d\n", port);
    return 0;
}

/* Idle: answer pages/API requests as they come, until someone opens /stream. */
static void http_wait_for_client() {
    printf("Waiting for a browser to connect...\n");
    g_status.streaming = 0;
    while (client_fd < 0 && g_nav_fd < 0 && !g_quit) {
        heartbeat();
        struct pollfd pfd;
        pfd.fd = server_fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        if (poll(&pfd, 1, 1000) > 0 && (pfd.revents & POLLIN)) http_accept_one();
    }
    g_status.streaming = client_fd >= 0;
}

/* libjpeg-turbo encode to an in-memory buffer via the standard
 * jpeg_mem_dest() API (part of libjpeg's public API since v8, real and
 * portable -- not the simplified TurboJPEG API, since jpeglib.h was
 * already the header on hand). */
/* One output buffer, allocated once with room for any frame (raw RGB size
 * is a safe upper bound at this quality), and its real capacity passed to
 * jpeg_mem_dest() every call.
 *
 * Real leak found 2026-10-11 (3-minute soak: process heap +200KB/s, ~14KB
 * per frame, same on Ubuntu): the old code passed back the previous call's
 * buffer with *outsize = the previous frame's COMPRESSED length, not the
 * buffer's capacity. Whenever a frame compressed larger than the one
 * before (about every other frame), libjpeg allocated a bigger buffer and
 * copied -- and never freed the old one, since a buffer passed in is the
 * caller's. */
static unsigned char* jpeg_buf = NULL;
static unsigned long jpeg_buf_cap = 0;

static unsigned long jpeg_encode_turbo(unsigned char* rgb, int w, int h) {
    if (!jpeg_buf) {
        jpeg_buf_cap = (unsigned long)w * h * 3 + 65536;
        jpeg_buf = (unsigned char*)malloc(jpeg_buf_cap);
    }
    /* size type differs: unsigned long* in libjpeg-turbo, size_t* in IJG 9 */
#if JPEG_LIB_VERSION >= 90
    size_t out_size = jpeg_buf_cap;
#else
    unsigned long out_size = jpeg_buf_cap;
#endif
    unsigned char* out = jpeg_buf;

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &out, &out_size);

    cinfo.image_width = w;
    cinfo.image_height = h;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 50, TRUE); /* same quality stb_image_write used, for a fair before/after */

    jpeg_start_compress(&cinfo, TRUE);
    int row_stride = w * 3;
    JSAMPROW row_pointer[1];
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &rgb[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    if (out != jpeg_buf) {
        /* only if a frame ever outgrew the buffer: libjpeg allocated a new
         * one (ours untouched); adopt it so nothing leaks */
        free(jpeg_buf);
        jpeg_buf = out;
        jpeg_buf_cap = out_size;
    }
    return out_size; /* compressed length */
}

/* Splits the combined "send_ms" this file used to report into its two
 * real, separate costs -- JPEG encode (CPU-bound, libjpeg-turbo since
 * 2026-09-11, stb_image_write before that -- see PROJECT_MEMORY.md)
 * and network write (send() to the connected client) -- an open
 * question this project had left unanswered across several sessions
 * (only the combined figure existed). out_jpeg_ms/out_net_ms report
 * each alone. */
#ifdef LUCKFOX_HW_JPEG
/* One DMA buffer the frame is copied into, encoded by VENC channel 0 in
 * single-picture JPEG mode. The finished JPEG is sent straight from the
 * encoder's output buffer (no copy). Input: NV12, converted from the HUD's
 * RGB on the CPU (full range, ~a few ms at 960x540).
 *
 * OPT-IN (LUCKFOX_HW_JPEG=1, set in /root/rknn_test/app.env), libjpeg
 * otherwise. Real results 2026-10-10 on this board:
 *  - RGB888 input (would skip the conversion) froze the whole board within
 *    seconds of startup, before any frame -- power cycle needed. Only tried
 *    now if LUCKFOX_VENC_FMT=rgb is set explicitly.
 *  - NV12 input: 9.0fps (JPEG 12-13.5ms incl. conversion) vs 5.4fps with
 *    IJG libjpeg and 8.5fps with libjpeg-turbo on Ubuntu.
 *  - killall mid-encode froze the board too; fixed by the graceful SIGTERM
 *    shutdown (g_quit, hwjpeg_shutdown) -- 5/5 stop-while-encoding rounds
 *    clean afterwards. */
#define HWJPEG_CHN 0
#define HWJPEG_QFACTOR 50 /* same quality setting libjpeg/stb used, for a fair comparison */
static bool g_hwjpeg = false;
static bool g_hwjpeg_nv12 = false;
static MB_BLK g_hwjpeg_blk = RK_NULL;
static rga_buffer_handle_t g_hwjpeg_rga_h = 0;   /* RGA import of g_hwjpeg_blk */
static VENC_STREAM_S g_hwjpeg_stream;

/* Full-range BT.601 (JFIF's YCbCr, 0-255), which is what the JPEG encoder
 * expects. Real result: limited-range (16-235) input here made the hardware
 * JPEGs look washed out -- blacks lifted to grey, low contrast. */
static inline unsigned char clamp_u8(int v) { return (unsigned char)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

static void rgb_to_nv12(const unsigned char* rgb, int w, int h, unsigned char* out) {
    unsigned char* y_plane = out;
    unsigned char* uv_plane = out + w * h;
    for (int j = 0; j < h; j++) {
        const unsigned char* p = rgb + j * w * 3;
        unsigned char* yr = y_plane + j * w;
        for (int i = 0; i < w; i++, p += 3) {
            yr[i] = (unsigned char)((77 * p[0] + 150 * p[1] + 29 * p[2] + 128) >> 8);
        }
    }
    for (int j = 0; j < h; j += 2) {
        const unsigned char* p = rgb + j * w * 3;
        unsigned char* uvr = uv_plane + (j / 2) * w;
        for (int i = 0; i < w; i += 2, p += 6) {
            int r = p[0], g = p[1], b = p[2];
            /* clamp: pure blue/red reach 256 with these coefficients */
            uvr[i] = clamp_u8(((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128);
            uvr[i + 1] = clamp_u8(((128 * r - 107 * g - 21 * b + 128) >> 8) + 128);
        }
    }
}

/* Step markers written into the kernel log, so they land in order with the
 * kernel's own messages -- and survive in ramoops (/sys/fs/pstore) if the
 * board freezes and the watchdog resets it (2026-10-11 freeze hunt). */
static void kmsg_mark(const char* step) {
    static int fd = -2;
    if (fd == -2) fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
    if (fd < 0) return;
    char line[160];
    int n = snprintf(line, sizeof(line), "<6>yolo-hwjpeg: %s\n", step);
    if (write(fd, line, n) < 0) { /* nothing useful to do */ }
}

static bool hwjpeg_create(int w, int h, PIXEL_FORMAT_E fmt) {
    VENC_CHN_ATTR_S attr;
    memset(&attr, 0, sizeof(attr));
    attr.stVencAttr.enType = RK_VIDEO_ID_JPEG;
    attr.stVencAttr.enPixelFormat = fmt;
    attr.stVencAttr.u32PicWidth = w;
    attr.stVencAttr.u32PicHeight = h;
    attr.stVencAttr.u32VirWidth = w;
    attr.stVencAttr.u32VirHeight = h;
    attr.stVencAttr.u32StreamBufCnt = 2;
    attr.stVencAttr.u32BufSize = w * h * 3 / 2;
    attr.stVencAttr.enMirror = MIRROR_NONE;
    attr.stVencAttr.stAttrJpege.bSupportDCF = RK_FALSE;
    attr.stVencAttr.stAttrJpege.stMPFCfg.u8LargeThumbNailNum = 0;
    attr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
    if (RK_MPI_VENC_CreateChn(HWJPEG_CHN, &attr) != RK_SUCCESS) return false;
    VENC_JPEG_PARAM_S jp;
    memset(&jp, 0, sizeof(jp));
    jp.u32Qfactor = HWJPEG_QFACTOR;
    RK_MPI_VENC_SetJpegParam(HWJPEG_CHN, &jp);
    VENC_RECV_PIC_PARAM_S recv;
    memset(&recv, 0, sizeof(recv));
    recv.s32RecvPicNum = -1; /* keep accepting frames */
    if (RK_MPI_VENC_StartRecvFrame(HWJPEG_CHN, &recv) != RK_SUCCESS) {
        RK_MPI_VENC_DestroyChn(HWJPEG_CHN);
        return false;
    }
    return true;
}

static void hwjpeg_init(int w, int h) {
    const char* opt = getenv("LUCKFOX_HW_JPEG");
    if (!opt || strcmp(opt, "1") != 0) { printf("RESULT: JPEG_MODE software (hardware is opt-in: LUCKFOX_HW_JPEG=1)\n"); return; }
    if (RK_MPI_SYS_Init() != RK_SUCCESS) { printf("RESULT: JPEG_MODE software (RK_MPI_SYS_Init failed)\n"); return; }
    const char* want = getenv("LUCKFOX_VENC_FMT");
    bool try_rgb = want && !strcmp(want, "rgb");
    if (try_rgb && hwjpeg_create(w, h, RK_FMT_RGB888)) {
        g_hwjpeg_nv12 = false;
    } else if (hwjpeg_create(w, h, RK_FMT_YUV420SP)) {
        g_hwjpeg_nv12 = true;
    } else {
        printf("RESULT: JPEG_MODE software (VENC channel create failed)\n");
        RK_MPI_SYS_Exit();
        return;
    }
    if (RK_MPI_SYS_MmzAlloc(&g_hwjpeg_blk, RK_NULL, RK_NULL, w * h * 3) != RK_SUCCESS) {
        printf("RESULT: JPEG_MODE software (MmzAlloc failed)\n");
        RK_MPI_VENC_StopRecvFrame(HWJPEG_CHN);
        RK_MPI_VENC_DestroyChn(HWJPEG_CHN);
        RK_MPI_SYS_Exit();
        return;
    }
    g_hwjpeg_stream.pstPack = (VENC_PACK_S*)malloc(sizeof(VENC_PACK_S));
    g_hwjpeg = true;
    kmsg_mark("init: VENC channel + buffer ready");
    printf("RESULT: JPEG_MODE hardware VENC input=%s q=%d\n", g_hwjpeg_nv12 ? "NV12(cpu-converted)" : "RGB888", HWJPEG_QFACTOR);
}

/* Encodes one frame; on success *data/*len point into the encoder's output
 * buffer until hwjpeg_release(). */
static bool hwjpeg_encode(const unsigned char* rgb, int w, int h, unsigned char** data, unsigned int* len) {
    unsigned char* dst = (unsigned char*)RK_MPI_MB_Handle2VirAddr(g_hwjpeg_blk);
    /* RGB -> NV12 on the RGA when the frame is one of its DMA-BUFs (1 ms vs
     * ~5 ms on the CPU), straight into the encoder's MB block (imported by
     * fd once). The CPU never writes the MB block on this path, so there are
     * no dirty cache lines to flush over the RGA's output. */
    static bool mb_tried = false;
    rga_buffer_handle_t& mb_h = g_hwjpeg_rga_h;
    RgaBuf* fb = (g_rga && g_hwjpeg_nv12) ? rga_frame_for(rgb) : NULL;
    if (fb && !mb_tried) {
        mb_tried = true;
        RK_MPI_SYS_MmzFlushCache(g_hwjpeg_blk, RK_FALSE);   /* nothing dirty left from the CPU path */
        im_handle_param_t p = { (uint32_t)w, (uint32_t)h, (uint32_t)RK_FORMAT_YCbCr_420_SP };
        int fd = RK_MPI_MB_Handle2Fd(g_hwjpeg_blk);
        if (fd >= 0) mb_h = importbuffer_fd(fd, &p);
        printf("RESULT: HWJPEG_INPUT %s\n", mb_h ? "RGA RGB->NV12" : "CPU RGB->NV12 (MB import failed)");
    }
    bool done = false;
    if (fb && mb_h) {
        dmabuf_sync(fb->fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);   /* HUD drawing -> memory */
        IM_STATUS st = imcvtcolor(wrapbuffer_handle(fb->h, w, h, RK_FORMAT_RGB_888),
                                  wrapbuffer_handle(mb_h, w, h, RK_FORMAT_YCbCr_420_SP),
                                  RK_FORMAT_RGB_888, RK_FORMAT_YCbCr_420_SP, IM_RGB_TO_YUV_BT601_LIMIT);
        dmabuf_sync(fb->fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW);
        if (st == IM_STATUS_SUCCESS || st == IM_STATUS_NOERROR) {
            done = true;
        } else {
            printf("RESULT: RGA_FAILED in JPEG input (%s), CPU colour conversion from now on\n", imStrError(st));
            g_rga = false;
        }
    }
    if (!done) {
        if (g_hwjpeg_nv12) rgb_to_nv12(rgb, w, h, dst);
        else memcpy(dst, rgb, (size_t)w * h * 3);
        RK_MPI_SYS_MmzFlushCache(g_hwjpeg_blk, RK_FALSE);
    }

    VIDEO_FRAME_INFO_S frame;
    memset(&frame, 0, sizeof(frame));
    frame.stVFrame.pMbBlk = g_hwjpeg_blk;
    frame.stVFrame.u32Width = w;
    frame.stVFrame.u32Height = h;
    frame.stVFrame.u32VirWidth = w;
    frame.stVFrame.u32VirHeight = h;
    frame.stVFrame.enPixelFormat = g_hwjpeg_nv12 ? RK_FMT_YUV420SP : RK_FMT_RGB888;
    if (RK_MPI_VENC_SendFrame(HWJPEG_CHN, &frame, 1000) != RK_SUCCESS) return false;
    if (RK_MPI_VENC_GetStream(HWJPEG_CHN, &g_hwjpeg_stream, 1000) != RK_SUCCESS) return false;
    *data = (unsigned char*)RK_MPI_MB_Handle2VirAddr(g_hwjpeg_stream.pstPack->pMbBlk);
    *len = g_hwjpeg_stream.pstPack->u32Len;
    return true;
}

static void hwjpeg_release(void) {
    RK_MPI_VENC_ReleaseStream(HWJPEG_CHN, &g_hwjpeg_stream);
}

static void hwjpeg_shutdown(void) {
    if (!g_hwjpeg_blk) return;
    if (g_hwjpeg_rga_h) { releasebuffer_handle(g_hwjpeg_rga_h); g_hwjpeg_rga_h = 0; }
    kmsg_mark("shutdown: begin");
    RK_MPI_VENC_StopRecvFrame(HWJPEG_CHN);
    kmsg_mark("shutdown: StopRecvFrame done");
    RK_MPI_VENC_DestroyChn(HWJPEG_CHN);
    kmsg_mark("shutdown: DestroyChn done");
    RK_MPI_SYS_MmzFree(g_hwjpeg_blk);
    kmsg_mark("shutdown: MmzFree done");
    g_hwjpeg_blk = RK_NULL;
    /* Real hang seen 2026-10-11: RK_MPI_SYS_Exit() blocked forever on a
     * futex inside librockit (process stuck holding port 8080, so the
     * restart loop never restarted it). The encoder channel is already
     * destroyed at this point, so the dangerous part is done: a 3s SIGALRM
     * (default action: terminate) guarantees the process still exits. */
    fflush(stdout);
    alarm(3);
    kmsg_mark("shutdown: SYS_Exit begin");
    RK_MPI_SYS_Exit();
    kmsg_mark("shutdown: SYS_Exit done");
    printf("RESULT: HW_JPEG_SHUTDOWN clean\n");
}
#endif

static int http_send_frame(unsigned char* rgb, int w, int h, double* out_jpeg_ms, double* out_net_ms) {
    struct timespec t0, t1, t2;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    unsigned char* jpeg_data = NULL;
    unsigned long jpeg_size = 0;
    bool hw = false;
#ifdef LUCKFOX_HW_JPEG
    sigset_t quit_sigs, old_sigs;
    sigemptyset(&quit_sigs);
    sigaddset(&quit_sigs, SIGTERM);
    sigaddset(&quit_sigs, SIGINT);
    bool sigs_blocked = g_hwjpeg;
    if (sigs_blocked) sigprocmask(SIG_BLOCK, &quit_sigs, &old_sigs);
    if (g_hwjpeg) {
        unsigned int len = 0;
        if (hwjpeg_encode(rgb, w, h, &jpeg_data, &len)) {
            jpeg_size = len;
            hw = true;
        } else {
            printf("RESULT: HW_JPEG_ENCODE_FAILED, switching to software for good\n");
            g_hwjpeg = false;
        }
    }
#endif
    if (!hw) {
        jpeg_size = jpeg_encode_turbo(rgb, w, h);
        jpeg_data = jpeg_buf;
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);

    char part_header[128];
    int hlen = snprintf(part_header, sizeof(part_header),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n", jpeg_size);
    int ret = 0;
    if (send_all(client_fd, part_header, hlen) < 0) ret = -1;
    if (ret == 0 && send_all(client_fd, jpeg_data, jpeg_size) < 0) ret = -1;
    if (ret == 0 && send_all(client_fd, "\r\n", 2) < 0) ret = -1;
#ifdef LUCKFOX_HW_JPEG
    if (hw) hwjpeg_release();
    if (sigs_blocked) sigprocmask(SIG_SETMASK, &old_sigs, NULL);
#endif

    clock_gettime(CLOCK_MONOTONIC, &t2);
    *out_jpeg_ms = (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6;
    *out_net_ms = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_nsec - t1.tv_nsec) / 1e6;
    return ret;
}

int main(int argc, char** argv) {
    /* Real bug found and fixed 2026-09-11 (live-camera integration
     * session): default SIGPIPE disposition kills the process on the
     * very FIRST failed send() to a disconnected client, before
     * http_send_frame()'s own return-value check (which already existed
     * to handle exactly this) ever runs -- confirmed via a foreground
     * run's real exit code (141 = 128+SIGPIPE), not guessed. Silently
     * masked in the video-file path until now because `ffmpeg`'s own
     * subprocess printed ITS OWN "Broken pipe" message when it hit
     * SIGPIPE on its now-orphaned stdout pipe, which looked like normal
     * shutdown noise; the live-camera path has no ffmpeg to mask it,
     * which is how this got caught for real. */
    signal(SIGPIPE, SIG_IGN);
    heartbeat(); /* first thing: a previous instance's stale beat must not look like a hang */
    {
        /* no SA_RESTART: a blocking accept()/capture returns EINTR so the
         * loops notice g_quit promptly */
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = on_quit_signal;
        sigaction(SIGTERM, &sa, NULL);
        sigaction(SIGINT, &sa, NULL);
    }

    if (argc < 3) {
        fprintf(stderr, "usage: %s model.rknn video.mp4-or-/dev/videoN [labels.txt] [http-port] [display_w] [display_h]\n", argv[0]);
        return 1;
    }
    const char* model_path = argv[1];
    const char* video_path = argv[2];
    const char* labels_path = argc > 3 ? argv[3] : "coco_80_labels_list.txt";
    int http_port = argc > 4 ? atoi(argv[4]) : HTTP_PORT_DEFAULT;
    setvbuf(stdout, NULL, _IOLBF, 0); /* line-buffer even when redirected to a file/log */
    load_labels(labels_path);
    if (getenv("LUCKFOX_WEB_ROOT")) g_web_root = getenv("LUCKFOX_WEB_ROOT");
    if (getenv("LUCKFOX_CAM_HFOV_DEG")) g_hfov_deg = atof(getenv("LUCKFOX_CAM_HFOV_DEG"));
    if (getenv("LUCKFOX_CAM_VFOV_DEG")) g_vfov_deg = atof(getenv("LUCKFOX_CAM_VFOV_DEG"));

    bool live_camera = (strncmp(video_path, "/dev/video", 10) == 0);

    int display_w = argc > 5 ? atoi(argv[5]) : 0;
    int display_h = argc > 6 ? atoi(argv[6]) : 0;
    if (display_w <= 0 || display_h <= 0) {
        if (live_camera) {
            /* benchmark default as of 2026-09-13 (see PROJECT_MEMORY.md):
             * real measured 8.4fps at 960x540 vs 4.3fps at 1920x1080, with
             * infer_ms unchanged either way since detection always runs on
             * a separate 640x640 downscale -- this is a display-quality/fps
             * tradeoff only, not a detection-accuracy one */
            display_w = 960; display_h = 540;
        } else {
            /* probe the source's own resolution via ffprobe rather than
             * guessing/hardcoding one */
            char probe_cmd[1024];
            snprintf(probe_cmd, sizeof(probe_cmd),
                     "ffprobe -v error -select_streams v:0 -show_entries stream=width,height "
                     "-of csv=p=0 '%s'", video_path);
            FILE* probe = popen(probe_cmd, "r");
            if (probe) {
                if (fscanf(probe, "%d,%d", &display_w, &display_h) != 2) { display_w = 0; display_h = 0; }
                pclose(probe);
            }
            if (display_w <= 0 || display_h <= 0) {
                fprintf(stderr, "RESULT: FFPROBE_FAIL, falling back to %dx%d\n", MODEL_SIZE, MODEL_SIZE);
                display_w = MODEL_SIZE; display_h = MODEL_SIZE;
            }
        }
    }

    V4L2Capture v4l2_cap;
#ifdef LUCKFOX_ROCKIT_VI
    RockitCam rk_cam;
    if (live_camera) {
        step_mark("start: opening Rockit VI camera (in-process 3A)");
        if (!rockit_cam_open(&rk_cam, display_w, display_h)) {
            rockit_cam_close(&rk_cam);
            return 1;
        }
        v4l2_cap.width = display_w;
        v4l2_cap.height = display_h;
    }
#else
    CamdClient camd;
    const char* camd_env = getenv("LUCKFOX_CAMD");
    const bool use_camd = live_camera && !(camd_env && camd_env[0] == '0');
    if (use_camd) {
        step_mark("start: connecting to yolo_camd (camera stream stays up)");
        if (!camd_open(&camd, &v4l2_cap)) {
            camd_close(&camd);
            return 1;
        }
    } else if (live_camera) {
        step_mark("start: opening V4L2 camera");
        if (!v4l2_open_and_configure(&v4l2_cap, video_path, display_w, display_h)) {
            printf("RESULT: V4L2_INIT_FAIL\n");
            return 1;
        }
    }
#endif
    if (live_camera) {
        /* the driver may have negotiated a different real size than
         * requested (stepwise 32x32, 8px steps on this sensor) -- use
         * what it actually reports from here on, not the request */
        display_w = v4l2_cap.width;
        display_h = v4l2_cap.height;
    }
    printf("RESULT: DISPLAY_RES %dx%d (inference size comes from the model)\n", display_w, display_h);
    g_status.display_w = display_w;
    g_status.display_h = display_h;
    g_status.source = video_path;
#ifdef LUCKFOX_HW_JPEG
    hwjpeg_init(display_w, display_h);
#endif

    /* --- RKNN init, zero-copy I/O, ONCE --- */
    rknn_context ctx = 0;
    step_mark("start: camera streaming; rknn_init (model load) begin");
    int ret = rknn_init(&ctx, (void*)model_path, 0, 0, NULL);
    step_mark("start: rknn_init done");
    if (ret != RKNN_SUCC) { printf("RESULT: INIT_FAIL ret=%d\n", ret); return 1; }

    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    rknn_tensor_attr input_attr;
    memset(&input_attr, 0, sizeof(input_attr));
    input_attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_NATIVE_INPUT_ATTR, &input_attr, sizeof(input_attr));
    input_attr.type = RKNN_TENSOR_INT8;
    input_attr.fmt = RKNN_TENSOR_NHWC;
    /* native input is NHWC: dims = {1, H, W, 3} */
    if (input_attr.n_dims == 4 && input_attr.dims[1] > 0 && input_attr.dims[2] > 0) {
        g_model_h = (int)input_attr.dims[1];
        g_model_w = (int)input_attr.dims[2];
    }
    const int model_row_stride = (input_attr.w_stride ? (int)input_attr.w_stride : g_model_w) * 3;
    printf("RESULT: MODEL_INPUT %dx%d (row stride %d px)\n", g_model_w, g_model_h, model_row_stride / 3);

    std::vector<rknn_tensor_attr> output_attrs(io_num.n_output);
    memset(output_attrs.data(), 0, sizeof(rknn_tensor_attr) * io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
        const rknn_tensor_attr& o = output_attrs[i];
        printf("RESULT: OUTPUT %u dims %ux%ux%ux%u zp %d scale %.6f fmt %d n_elems %u size %u size_with_stride %u w_stride %u h_stride %u\n", i,
               o.dims[0], o.dims[1], o.dims[2], o.dims[3], o.zp, o.scale, (int)o.fmt, o.n_elems, o.size,
               o.size_with_stride, o.w_stride, o.h_stride);
    }

    step_mark("start: rknn_create_mem (NPU buffers) begin");
    rknn_tensor_mem* input_mem = rknn_create_mem(ctx, input_attr.size_with_stride);
    rknn_set_io_mem(ctx, input_mem, &input_attr);
    std::vector<rknn_tensor_mem*> output_mems(io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_mems[i] = rknn_create_mem(ctx, output_attrs[i].size_with_stride);
        rknn_set_io_mem(ctx, output_mems[i], &output_attrs[i]);
    }
    rknn_tensor_mem* input_mem2 = rknn_create_mem(ctx, input_attr.size_with_stride);
    rknn_tensor_mem* input_mem3 = rknn_create_mem(ctx, input_attr.size_with_stride);
    NpuWorker npu;
    npu.ctx = ctx;
    npu.input_mems[0] = input_mem;
    npu.input_mems[1] = input_mem2;
    npu.input_mems[2] = input_mem3;
    npu.input_attr = input_attr;
    npu.output_attrs = &output_attrs;
    npu.output_mems = &output_mems;
    pthread_mutex_init(&npu.lock, NULL);
    pthread_cond_init(&npu.cond, NULL);
    npu.job_idx = -1;
    npu.next_idx = -1;
    npu.job_done = true;
    npu.quit = false;
    npu.done_seq = 0;
    npu.last_done = -1;
    /* The worker must never take a signal: SIGTERM (killall, deploy,
     * reboot) can be delivered to any thread that doesn't block it, and one
     * landing on the worker interrupts its wait inside the NPU driver --
     * the likely trigger of the NPU hangs and board freezes seen
     * 2026-10-11. The thread inherits this mask; main restores its own. */
    sigset_t block_sigs, old_sigs;
    sigemptyset(&block_sigs);
    sigaddset(&block_sigs, SIGTERM);
    sigaddset(&block_sigs, SIGINT);
    sigaddset(&block_sigs, SIGPIPE);
    sigaddset(&block_sigs, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &block_sigs, &old_sigs);
    pthread_t npu_thread;
    int thread_err = pthread_create(&npu_thread, NULL, npu_worker_main, &npu);
    pthread_sigmask(SIG_SETMASK, &old_sigs, NULL);
    if (thread_err != 0) {
        printf("RESULT: NPU_THREAD_FAIL\n");
        return 1;
    }
    printf("RESULT: MODEL_READY n_output=%d\n", io_num.n_output);
    step_mark("start: NPU buffers + worker ready");

    if (http_start_server(http_port) != 0) return 1;
    step_mark("start: HTTP listening, idle until a viewer/subscriber connects");
    heartbeat();
    /* The hardware watchdog is owned by yolo_supervisor (started at boot), not
     * this process: the server disarming it on exit, and then failing to
     * restart, left a hung board unprotected (2026-10-11). The supervisor
     * watches this process's heartbeat instead. watchdog_arm() stays only for
     * running the server stand-alone without the supervisor. */
    if (getenv("LUCKFOX_SERVER_OWNS_WATCHDOG")) watchdog_arm();

    AccelState accel;
    accel_discover(&accel);
    g_accel = &accel;
    level_load();   /* mounting calibration from the web UI's Set level button */

    /* env-controlled, not a new CLI arg. Defaults OFF: real, measured
     * result this session is that RGA-offloaded downscale, even in its
     * best (memcpy-bounce) working form, is SLOWER than the original
     * CPU nearest-neighbor loop on this exact hardware (~46-57ms vs.
     * ~18-26ms) -- the mandatory copy-through-contiguous-DMA-memory tax
     * (this chip's RGA has no IOMMU, needs physically-contiguous
     * buffers, which this driver maps CPU-uncached -- see
     * downscale_for_inference_rga's comment for the full real
     * investigation) outweighs what the hardware resize itself saves,
     * for an operation this cheap on the CPU to begin with. Kept
     * available via USE_RGA_DOWNSCALE=1 for anyone who wants to
     * re-verify or build on this finding, not deleted -- a real,
     * working, just not currently worthwhile code path. */
    bool use_rga_downscale = (getenv("USE_RGA_DOWNSCALE") != NULL);
    /* three full-res frames: one being captured, one in the NPU, one being
     * drawn and encoded (see NPU worker) */
    unsigned char* frames[3];
    for (int i = 0; i < 3; i++) frames[i] = (unsigned char*)malloc(display_w * display_h * 3);
    unsigned char* frame_full = frames[0];
    unsigned char* frame_small = (unsigned char*)malloc(g_model_w * g_model_h * 3);
#ifndef LUCKFOX_ROCKIT_VI
    unsigned char* malloc_frames[3] = { frames[0], frames[1], frames[2] };
    unsigned char* malloc_small = frame_small;
    if (use_camd && rga_init(&camd, display_w, display_h, g_model_w, g_model_h, frames, &frame_small)) {
        for (int i = 0; i < 3; i++) free(malloc_frames[i]);
        free(malloc_small);
        use_rga_downscale = false;   /* the old DRM-buffer RGA downscale is not needed */
    }
#endif
    if (use_rga_downscale) {
        if (drm_init()) {
            g_drm_src = (unsigned char*)drm_alloc_contig(display_w, display_h, 3, &g_src_stride_px);
            g_drm_dst = (unsigned char*)drm_alloc_contig(g_model_w, g_model_h, 3, &g_dst_stride_px);
        }
        if (!g_drm_src || !g_drm_dst) {
            printf("RESULT: DRM_ALLOC_FAILED, falling back to CPU downscale\n");
            use_rga_downscale = false;
        }
    }
    float scale_x = (float)display_w / g_model_w;
    float scale_y = (float)display_h / g_model_h;
    printf("RESULT: DOWNSCALE_MODE %s\n", use_rga_downscale ? "RGA" : "CPU");

    /* HUD drawing + JPEG + send for one frame, shared by the pipelined loop
     * and the decoupled-display loop. Draws dets (in model coordinates) onto
     * frame_full; does nothing without a viewer. */
    auto hud_send = [&](unsigned char* frame_full, std::vector<Detection>& dets, double infer_ms,
                        int frame_count, double fps, float cpu_pct, int has_tilt, float tilt_pitch,
                        float tilt_roll, double& hud_ms, double& jpeg_ms, double& net_ms) {
        image_buffer_t img;
        memset(&img, 0, sizeof(img));
        img.width = display_w; img.height = display_h;
        img.width_stride = display_w; img.height_stride = display_h;
        img.format = IMAGE_FORMAT_RGB888;
        img.virt_addr = frame_full;
        img.size = display_w * display_h * 3;
        struct timespec t_hud0, t_hud1;
        clock_gettime(CLOCK_MONOTONIC, &t_hud0);
        if (client_fd >= 0) {

        if (!g_view_hud) {
            draw_plain_overlay(&img, dets, scale_x, scale_y);
        } else {
            /* apex_hud port (see PROJECT_MEMORY.md 2026-09-13) --
             * CrosshairWidget (static), TrackBoxesWidget (real detections,
             * reduced to bbox+class+confidence -- no azimuth/distance,
             * that needs #32's mono-distance work), RadarMapWidget (honest
             * grey stub, no real per-object x/y/z yet). */
            apex_hud_draw_crosshair(&img);
            {
                std::vector<HudTrack> hud_tracks;
                hud_tracks.reserve(dets.size());
                for (auto& d : dets) {
                    HudTrack t;
                    t.x = d.x; t.y = d.y; t.w = d.w; t.h = d.h; t.score = d.score;
                    t.class_name = class_names[d.cls_id];
                    hud_tracks.push_back(t);
                }
                apex_hud_draw_track_boxes(&img, hud_tracks.data(), (int)hud_tracks.size(), scale_x, scale_y);
            }
            {
                HudBounds radar_box;
                radar_box.x1 = std::max((int)(display_w * 0.815f), 4);
                radar_box.y1 = std::max((int)(display_h * 0.775f) + 4, 4);
                radar_box.x2 = display_w - 4;
                radar_box.y2 = std::min((int)(display_h * 0.93f), display_h - 4);
                apex_hud_draw_radar_map_stub(&img, radar_box);
            }
            /* Neutral/grey stubs -- this board has no orientation/heading
             * source (MPU-6050 raw accel only). Matches apex_hud's own
             * real "no imu_reading" default for each of these, per
             * PROJECT_MEMORY.md 2026-09-13 "apex_hud: full HUD pass". */
            /* Pitch/roll from the MPU-6050's smoothed gravity vector
             * (2026-10-11; previously grey stubs). Heading needs a
             * magnetometer/gyro integration this board doesn't have, so
             * the compass tape stays a stub. */
            {
                HudBounds cluster_box = apex_hud_cluster_bounds(&img);
                apex_hud_draw_horizon(&img, cluster_box, has_tilt, tilt_pitch, tilt_roll);
                apex_hud_draw_roll_ring(&img, cluster_box, has_tilt, tilt_roll);
            }
            {
                HudBounds frame_box = apex_hud_frame_bounds(&img);
                apex_hud_draw_compass_tape_stub(&img, frame_box, 0);
                apex_hud_draw_attitude_readouts(&img, frame_box, has_tilt, tilt_pitch, tilt_roll);
            }

            /* always-visible HUD readout panel, top-left -- apex_hud's
             * "always draw the instrument, neutral default when idle"
             * convention, loosely */
            int diag_fsz = apex_hud_font_size(&img, 7);
            int diag_line_h = diag_fsz + 4;
            int diag_panel_h = diag_line_h * 4 + 8;
            int diag_panel_w = (int)(220 * img.height / 540.0f + 0.5f);
            fill_rect(frame_full, display_w, display_h, 4, 4, diag_panel_w, diag_panel_h, 15, 15, 15);
            char hud_line1[64], hud_line2[64], hud_line3[64], hud_line4[64];
            snprintf(hud_line1, 64, "frame %d  fps %.1f", frame_count, fps);
            snprintf(hud_line2, 64, "infer %.1fms  det %zu", infer_ms, dets.size());
            snprintf(hud_line3, 64, "cpu %.0f%%", cpu_pct);
            snprintf(hud_line4, 64, "LUCKFOX RV1106 / yolov8n @ %dx%d disp", display_w, display_h);
            draw_text(&img, hud_line1, 8, 8, COLOR_WHITE, diag_fsz);
            draw_text(&img, hud_line2, 8, 8 + diag_line_h, COLOR_YELLOW, diag_fsz);
            draw_text(&img, hud_line3, 8, 8 + diag_line_h * 2, COLOR_ORANGE, diag_fsz);
            draw_text(&img, hud_line4, 8, 8 + diag_line_h * 3, COLOR_GREEN, diag_fsz);

            /* accelerometer readout panel, below the first -- always
             * drawn (apex_hud convention), grey "not connected" text
             * until a real MPU-6050 is actually found and reading */
            {
                HudBounds accel_box;
                accel_box.x1 = 4;
                accel_box.y1 = std::max((int)(display_h * 0.775f) + 4, 4);
                accel_box.x2 = std::max((int)(display_w * 0.20f) - 4, 20);
                accel_box.y2 = display_h - 4;
                int has_accel = accel.found && accel.ema_initialized;
                apex_hud_draw_accelerometer(&img, accel_box, has_accel,
                                             accel.smoothed_x * 9.81f, accel.smoothed_y * 9.81f);
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &t_hud1);
        hud_ms = (t_hud1.tv_sec - t_hud0.tv_sec) * 1000.0 + (t_hud1.tv_nsec - t_hud0.tv_nsec) / 1e6;
        if (http_send_frame(frame_full, display_w, display_h, &jpeg_ms, &net_ms) < 0) {
            printf("RESULT: CLIENT_DISCONNECTED after %d frames\n", frame_count);
            close(client_fd);
            client_fd = -1;
            g_status.streaming = 0;
            session_reset(); /* per-session object log is wiped on disconnect */
        } else {
            g_status.jpeg_ms = roll_add(&g_roll_jpeg, jpeg_ms);
        }
        } /* client_fd >= 0: HUD + JPEG only when someone is watching */
    };

#ifndef LUCKFOX_ROCKIT_VI
    /* Decoupled display (2026-10-12), used while a viewer is watching and the
     * RGA path is on: every iteration shows the NEWEST camera frame (paced by
     * camd, i.e. the camera), with the boxes from the latest finished
     * detection. Whenever the NPU is idle, the same frame is also resized by
     * the RGA and submitted as the next detection, without waiting for it.
     * Video is then limited by the camera/CPU instead of the NPU's 58 ms; boxes
     * lag the picture by one detection. The object log and navigation output
     * update once per detection. Navigation-only clients keep the pipelined
     * loop. LUCKFOX_DECOUPLED=0: pipelined loop for viewers too. */
    const char* dec_env = getenv("LUCKFOX_DECOUPLED");
    bool decoupled_ok = live_camera && use_camd && !(dec_env && dec_env[0] == '0');
    auto run_decoupled = [&]() -> bool {   /* false: use the pipelined loop from now on */
        printf("RESULT: DISPLAY_MODE decoupled (video at camera rate, detection at NPU rate)\n");
        std::vector<Detection> shown;
        std::vector<int> shown_ids;
        /* Three NPU input tensors: the worker runs one, may have one staged
         * (it starts that itself the moment the running job ends), and this
         * loop writes the newest frame into the third, then stages it. */
        bool fresh = false;
        int n_frames = 0, n_dets = 0;
        double quant_ms = 0, npu_ms = 0, post_ms = 0;
        bool run_failed = false;
        pthread_mutex_lock(&npu.lock);
        unsigned seen_seq = npu.done_seq;
        pthread_mutex_unlock(&npu.lock);
        struct timespec t_start, t_now;
        clock_gettime(CLOCK_MONOTONIC, &t_start);
        std::function<void()> collect = [&]() {
            pthread_mutex_lock(&npu.lock);
            if (npu.done_seq != seen_seq) {
                n_dets += (int)(npu.done_seq - seen_seq);
                seen_seq = npu.done_seq;
                const int d = npu.last_done;
                if (npu.ret[d] != RKNN_SUCC) {
                    printf("RESULT: RUN_FAIL ret=%d\n", npu.ret[d]);
                    run_failed = true;
                } else {
                    shown = npu.dets[d];
                    npu_ms = npu.npu_ms[d];
                    post_ms = npu.post_ms[d];
                    fresh = true;
                }
            }
            pthread_mutex_unlock(&npu.lock);
        };
        while (client_fd >= 0 && !g_quit) {
            heartbeat();
            collect();
            if (run_failed) { npu_drain(&npu); return false; }
            struct timespec r0, r1;
            clock_gettime(CLOCK_MONOTONIC, &r0);
            if (!camd_capture_frame_rga(&camd, 0, display_w, display_h, g_model_w, g_model_h, true, &collect)) {
                npu_drain(&npu);
                return false;
            }
            clock_gettime(CLOCK_MONOTONIC, &r1);
            double read_ms = (r1.tv_sec - r0.tv_sec) * 1000.0 + (r1.tv_nsec - r0.tv_nsec) / 1e6;
            {
                /* the newest frame into the tensor that is neither running nor staged */
                pthread_mutex_lock(&npu.lock);
                const int busy = npu.job_idx, queued = npu.next_idx;
                pthread_mutex_unlock(&npu.lock);
                int slot = 0;
                while (slot == busy || slot == queued) slot++;
                struct timespec q0, q1;
                clock_gettime(CLOCK_MONOTONIC, &q0);
                int8_t* in_buf = (int8_t*)npu.input_mems[slot]->virt_addr;
                const uint8x16_t k80 = vdupq_n_u8(0x80);
                for (int y = 0; y < g_model_h; y++) {
                    const unsigned char* src_row = g_rga_small.va + (size_t)y * g_model_w * 3;
                    uint8_t* dst_row = (uint8_t*)in_buf + (size_t)y * model_row_stride;
                    const int n = g_model_w * 3;
                    int i = 0;
                    for (; i + 16 <= n; i += 16) vst1q_u8(dst_row + i, veorq_u8(vld1q_u8(src_row + i), k80));
                    for (; i < n; i++) dst_row[i] = src_row[i] ^ 0x80;
                }
                clock_gettime(CLOCK_MONOTONIC, &q1);
                quant_ms = (q1.tv_sec - q0.tv_sec) * 1000.0 + (q1.tv_nsec - q0.tv_nsec) / 1e6;
                npu_stage(&npu, slot);
            }

            accel_update(&accel);
            int has_tilt;
            float tilt_pitch, tilt_roll;
            accel_tilt(&accel, &has_tilt, &tilt_pitch, &tilt_roll);
            if (fresh) {
                fresh = false;
                session_update(shown, &shown_ids);
                g_last_det_json = build_det_json(n_dets, shown, shown_ids, display_w, display_h,
                                                 has_tilt, tilt_pitch, tilt_roll);
                if (g_nav_fd >= 0) {
                    std::string ev = "data: " + g_last_det_json + "\n\n";
                    if (send_all(g_nav_fd, ev.data(), ev.size()) < 0) {
                        printf("RESULT: NAV_SUBSCRIBER_DISCONNECTED\n");
                        close(g_nav_fd);
                        g_nav_fd = -1;
                    }
                }
            }

            n_frames++;
            clock_gettime(CLOCK_MONOTONIC, &t_now);
            double elapsed = (t_now.tv_sec - t_start.tv_sec) + (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
            double fps = elapsed > 0 ? n_frames / elapsed : 0;
            double det_hz = elapsed > 0 ? n_dets / elapsed : 0;
            double infer_ms = quant_ms + npu_ms + post_ms;
            float cpu_pct = read_cpu_percent();
            g_status.frame = n_frames;
            g_status.fps = fps;
            g_status.det_hz = det_hz;
            g_status.infer_ms = roll_add(&g_roll_infer, infer_ms);
            g_status.read_ms = roll_add(&g_roll_read, read_ms);
            g_status.dets = (int)shown.size();
            g_status.cpu_pct = cpu_pct;
            g_status.has_tilt = has_tilt;
            g_status.pitch = tilt_pitch;
            g_status.roll = tilt_roll;
            g_status.streaming = client_fd >= 0;

            double hud_ms = 0, jpeg_ms = 0, net_ms = 0;
            hud_send(frames[0], shown, infer_ms, n_frames, fps, cpu_pct, has_tilt, tilt_pitch, tilt_roll,
                     hud_ms, jpeg_ms, net_ms);
            http_poll_requests();
            if (n_frames % 30 == 0) {
                printf("decoupled frame=%d fps=%.1f det_hz=%.1f read_ms=%.1f quant=%.1f npu=%.1f post=%.1f "
                       "hud_ms=%.1f jpeg_ms=%.1f net_ms=%.1f dets=%zu\n",
                       n_frames, fps, det_hz, read_ms, quant_ms, npu_ms, post_ms, hud_ms, jpeg_ms, net_ms, shown.size());
                fflush(stdout);
            }
        }
        npu_drain(&npu);
        return true;
    };
#endif

    while (!g_quit) {
        http_wait_for_client();
        if (g_quit) break;

        FILE* ffmpeg_pipe = NULL;
        if (!live_camera) {
            char ffmpeg_cmd[1024];
            /* Decode/scale to the FULL display resolution -- no -re (don't
             * throttle to source fps, consume as fast as this pipeline
             * can go); fast_bilinear (cheap, real CPU saving on this
             * single-core chip vs the default bicubic). The separate
             * MODEL_SIZE downscale for inference happens in-process below. */
            snprintf(ffmpeg_cmd, sizeof(ffmpeg_cmd),
                     "ffmpeg -i '%s' -loglevel error -vf scale=%d:%d:flags=fast_bilinear -pix_fmt rgb24 -f rawvideo -",
                     video_path, display_w, display_h);
            ffmpeg_pipe = popen(ffmpeg_cmd, "r");
            if (!ffmpeg_pipe) { printf("RESULT: FFMPEG_SPAWN_FAIL\n"); break; }
        }

        int frame_count = 0;
        struct timespec fps_t0, now;
        clock_gettime(CLOCK_MONOTONIC, &fps_t0);
        /* Three-stage pipeline over frame buffers 0..2: ready (captured +
         * prepared, waiting for the NPU), in flight (in the NPU), display
         * (inferred, being drawn/encoded/sent). Each iteration: hand the
         * ready frame to the NPU, capture+prepare the next one while it
         * runs, display the previous one, then wait for the NPU. */
        int ready_fb = -1, infl_fb = -1, disp_fb = -1;
        int fb_slot[3] = {0, 0, 0};  /* which NPU input tensor each frame uses */
        double fb_read_ms[3] = {0, 0, 0}, fb_ds_ms[3] = {0, 0, 0}, fb_quant_ms[3] = {0, 0, 0};
        bool stop = false;

        while ((client_fd >= 0 || g_nav_fd >= 0) && !g_quit) {
            heartbeat();
#ifndef LUCKFOX_ROCKIT_VI
            if (decoupled_ok && g_rga && client_fd >= 0) {
                /* a viewer is watching: drain the pipelined loop, then run the
                 * decoupled one until the viewer leaves */
                if (infl_fb >= 0) npu_wait(&npu);
                ready_fb = infl_fb = disp_fb = -1;
                if (!run_decoupled()) {
                    printf("RESULT: DISPLAY_MODE pipelined (decoupled loop stopped)\n");
                    decoupled_ok = false;
                }
                continue;
            }
#endif
            /* 1. hand the prepared frame to the NPU */
            if (ready_fb >= 0 && infl_fb < 0) {
                static bool first_job_marked = false;
                if (!first_job_marked) { step_mark("run: first NPU job submitted"); first_job_marked = true; }
                npu_submit(&npu, fb_slot[ready_fb]);
                infl_fb = ready_fb;
                ready_fb = -1;
            }

            /* 2. capture + prepare the next frame while the NPU runs: a
             * frame buffer not in flight or awaiting display, and the NPU
             * input tensor the in-flight frame isn't using */
            if (ready_fb < 0 && !stop) {
                int fb = 0;
                while (fb == infl_fb || fb == disp_fb) fb++;
                const int slot = (infl_fb >= 0) ? (fb_slot[infl_fb] ^ 1) : 0;
                struct timespec t_read_start, t_read_end;
                clock_gettime(CLOCK_MONOTONIC, &t_read_start);
                bool got_frame = false;
                bool small_from_rga = false;   /* RGA already made frame_small */
                unsigned char* cap_frame = frames[fb];
                if (live_camera) {
#ifdef LUCKFOX_ROCKIT_VI
                    got_frame = rockit_cam_capture(&rk_cam, cap_frame);
#else
                    if (use_camd && g_rga) {
                        got_frame = camd_capture_frame_rga(&camd, fb, display_w, display_h, g_model_w, g_model_h, true);
                        small_from_rga = got_frame;
                    }
                    if (!small_from_rga)
                        got_frame = use_camd ? camd_capture_frame(&camd, display_w, display_h, cap_frame)
                                             : v4l2_capture_frame(&v4l2_cap, cap_frame);
#endif
                } else {
                    size_t n = fread(cap_frame, 1, display_w * display_h * 3, ffmpeg_pipe);
                    got_frame = (n == (size_t)(display_w * display_h * 3));
                }
                clock_gettime(CLOCK_MONOTONIC, &t_read_end);
                double read_ms = (t_read_end.tv_sec - t_read_start.tv_sec) * 1000.0 +
                                  (t_read_end.tv_nsec - t_read_start.tv_nsec) / 1e6;
                if (!got_frame) {
                    if (live_camera) printf("RESULT: V4L2_CAPTURE_FAIL\n");
                    stop = true; /* video ended, or a real capture error */
                } else {

                struct timespec t_ds_start, t_ds_end;
                clock_gettime(CLOCK_MONOTONIC, &t_ds_start);
                if (small_from_rga) {
                    /* done by the RGA during capture (counted in read_ms) */
                } else if (use_rga_downscale) {
                    if (!downscale_for_inference_rga(cap_frame, display_w, display_h, frame_small, g_model_w, g_model_h)) {
                        fprintf(stderr, "RESULT: RGA_DOWNSCALE_FAILED, falling back to CPU downscale for this and all future frames\n");
                        use_rga_downscale = false;
                        downscale_for_inference(cap_frame, display_w, display_h, frame_small, g_model_w, g_model_h);
                    }
                } else {
                    downscale_for_inference(cap_frame, display_w, display_h, frame_small, g_model_w, g_model_h);
                }
                clock_gettime(CLOCK_MONOTONIC, &t_ds_end);
                double downscale_ms = (t_ds_end.tv_sec - t_ds_start.tv_sec) * 1000.0 +
                                       (t_ds_end.tv_nsec - t_ds_start.tv_nsec) / 1e6;
                struct timespec t_q0, t_q1;
                clock_gettime(CLOCK_MONOTONIC, &t_q0);
                int8_t* in_buf = (int8_t*)npu.input_mems[slot]->virt_addr;
                /* uint8 -> int8 with zero point -128: x - 128 has the same bits
                 * as x ^ 0x80, 16 bytes per NEON instruction */
                const uint8x16_t k80 = vdupq_n_u8(0x80);
                for (int y = 0; y < g_model_h; y++) {
                    const unsigned char* src_row = frame_small + (size_t)y * g_model_w * 3;
                    uint8_t* dst_row = (uint8_t*)in_buf + (size_t)y * model_row_stride;
                    const int n = g_model_w * 3;
                    int i = 0;
                    for (; i + 16 <= n; i += 16) vst1q_u8(dst_row + i, veorq_u8(vld1q_u8(src_row + i), k80));
                    for (; i < n; i++) dst_row[i] = src_row[i] ^ 0x80;
                }
                clock_gettime(CLOCK_MONOTONIC, &t_q1);
                fb_read_ms[fb] = read_ms;
                fb_ds_ms[fb] = downscale_ms;
                fb_quant_ms[fb] = (t_q1.tv_sec - t_q0.tv_sec) * 1000.0 + (t_q1.tv_nsec - t_q0.tv_nsec) / 1e6;
                fb_slot[fb] = slot;
                ready_fb = fb;
                } /* got_frame */
            }

            /* 3. draw / encode / send the previous frame with its own detections */
            if (disp_fb >= 0) {
            const int dslot = fb_slot[disp_fb];
            unsigned char* frame_full = frames[disp_fb];
            std::vector<Detection>& dets = npu.dets[dslot];
            double read_ms = fb_read_ms[disp_fb];
            double downscale_ms = fb_ds_ms[disp_fb];
            double quant_ms = fb_quant_ms[disp_fb];
            double npu_ms = npu.npu_ms[dslot];
            double infer_ms = downscale_ms + quant_ms + npu_ms + npu.post_ms[dslot];

            accel_update(&accel);

            /* --- draw HUD overlay onto the FULL-res frame -- box
             * coordinates rescaled from MODEL_SIZE space up to
             * display_w x display_h --- */

            frame_count++;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (now.tv_sec - fps_t0.tv_sec) + (now.tv_nsec - fps_t0.tv_nsec) / 1e9;
            double fps = elapsed > 0 ? frame_count / elapsed : 0;
            float cpu_pct = read_cpu_percent();
            g_status.frame = frame_count;
            g_status.fps = fps;
            g_status.det_hz = fps;   /* pipelined loop: one detection per displayed frame */
            g_status.infer_ms = roll_add(&g_roll_infer, infer_ms);
            g_status.read_ms = roll_add(&g_roll_read, read_ms);
            g_status.dets = (int)dets.size();
            g_status.cpu_pct = cpu_pct;
            std::vector<int> det_ids;
            session_update(dets, &det_ids);

            /* tilt from gravity: roll about the sensor X axis, pitch about Y
             * (standard accelerometer-only formulas; valid while the board
             * isn't accelerating hard -- the EMA smoothing helps there) */
            int has_tilt;
            float tilt_pitch, tilt_roll;
            accel_tilt(&accel, &has_tilt, &tilt_pitch, &tilt_roll);
            g_status.has_tilt = has_tilt;
            g_status.pitch = tilt_pitch;
            g_status.roll = tilt_roll;
            g_status.streaming = client_fd >= 0;

            /* navigation output: every frame, viewer or not */
            g_last_det_json = build_det_json(frame_count, dets, det_ids, display_w, display_h,
                                             has_tilt, tilt_pitch, tilt_roll);
            if (g_nav_fd >= 0) {
                std::string ev = "data: " + g_last_det_json + "\n\n";
                if (send_all(g_nav_fd, ev.data(), ev.size()) < 0) {
                    printf("RESULT: NAV_SUBSCRIBER_DISCONNECTED\n");
                    close(g_nav_fd);
                    g_nav_fd = -1;
                }
            }

            double jpeg_ms = 0, net_ms = 0, hud_ms = 0;
            hud_send(frame_full, dets, infer_ms, frame_count, fps, cpu_pct, has_tilt, tilt_pitch, tilt_roll,
                     hud_ms, jpeg_ms, net_ms);
            http_poll_requests();

            if (frame_count % 10 == 0) {
                printf("frame=%d fps=%.1f read_ms=%.1f downscale_ms=%.1f infer_ms=%.1f (quant=%.1f npu=%.1f post=%.1f post_cpu=%.1f copy=%.1f) "
                       "hud_ms=%.1f jpeg_ms=%.1f net_ms=%.1f dets=%zu\n",
                       frame_count, fps, read_ms, downscale_ms, infer_ms, quant_ms, npu_ms,
                       infer_ms - quant_ms - npu_ms - downscale_ms, npu.post_cpu_ms[dslot], npu.copy_ms[dslot], hud_ms, jpeg_ms, net_ms, dets.size());
                fflush(stdout);
            }
            disp_fb = -1;
            } /* disp_fb */

            /* 4. wait for the NPU; that frame is displayed next iteration */
            if (infl_fb >= 0) {
                npu_wait(&npu);
                static bool first_done_marked = false;
                if (!first_done_marked) { step_mark("run: first NPU job done"); first_done_marked = true; }
                if (npu.ret[fb_slot[infl_fb]] != RKNN_SUCC) {
                    printf("RESULT: RUN_FAIL ret=%d\n", npu.ret[fb_slot[infl_fb]]);
                    break;
                }
                disp_fb = infl_fb;
                infl_fb = -1;
            }
            if (stop && infl_fb < 0) break;
        }

        if (ffmpeg_pipe) pclose(ffmpeg_pipe);
        if (client_fd >= 0) { close(client_fd); client_fd = -1; }
        g_status.streaming = 0;
        g_last_det_json = "{\"active\":false}";
        session_reset(); /* per-session object log is wiped on disconnect */
    }

    printf("RESULT: SHUTTING_DOWN\n");
    if (client_fd >= 0) { close(client_fd); client_fd = -1; }
    /* Camera first, Rockit last. Real freeze 2026-10-11 (kernel log streamed
     * to the Mac): RK_MPI_SYS_Exit() closes every Rockit device, and the
     * kernel locked up right after Rockit's vi_release -- while this
     * process's own V4L2 stream on the same ISP was still running. So the
     * V4L2 stream is fully stopped and closed before Rockit is torn down. */
#ifdef LUCKFOX_ROCKIT_VI
    if (live_camera) rockit_cam_close(&rk_cam);
#else
    if (use_camd) {
        rga_shutdown();      /* release RGA imports of camd's buffers first */
        camd_close(&camd);   /* the stream itself keeps running in yolo_camd */
    } else if (live_camera) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(v4l2_cap.fd, VIDIOC_STREAMOFF, &type);
        for (int i = 0; i < v4l2_cap.n_buffers; i++) munmap(v4l2_cap.buffers[i].start, v4l2_cap.buffers[i].length);
        close(v4l2_cap.fd);
    }
#endif
#ifdef LUCKFOX_HW_JPEG
    if (g_hwjpeg_blk) kmsg_mark("main: v4l2 stream closed, Rockit teardown next");
    hwjpeg_shutdown();
#endif
    pthread_mutex_lock(&npu.lock);
    npu.quit = true;
    pthread_cond_broadcast(&npu.cond);
    pthread_mutex_unlock(&npu.lock);
    pthread_join(npu_thread, NULL);
    for (int i = 0; i < 3; i++) free(frames[i]);
    free(frame_small);
    for (uint32_t i = 0; i < io_num.n_output; i++) rknn_destroy_mem(ctx, output_mems[i]);
    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy_mem(ctx, input_mem2);
    rknn_destroy_mem(ctx, input_mem3);
    rknn_destroy(ctx);
    unlink(HEARTBEAT_FILE); /* clean exit: not a hang */
    watchdog_disarm();      /* last of all: a teardown that hangs still resets the board */
#ifdef LUCKFOX_HW_JPEG
    if (g_hwjpeg_nv12) kmsg_mark("main: teardown done, exiting");
#endif
    return 0;
}
