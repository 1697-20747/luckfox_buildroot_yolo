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
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>
#include <string>
#include <vector>
#include <algorithm>
#include <set>

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
#ifdef LUCKFOX_HW_JPEG
#include "rk_mpi_sys.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_venc.h"
#endif

#define OBJ_CLASS_NUM 80
#define NUM_SCALES 3
#define BOX_THRESH 0.25f
#define NMS_THRESH 0.45f
#define MODEL_SIZE 640
#define HTTP_PORT_DEFAULT 8080

static const int STRIDES[NUM_SCALES] = {8, 16, 32};

/* 1697-20747: "object types can be restricted for now to speed inference as
 * needed" -- COCO class ids relevant to the linked test videos
 * (bikes/cars/motorbikes/people): person=0, bicycle=1, car=2,
 * motorcycle=3, bus=5, truck=7. Skipping the other 74 classes in the
 * decode loop cuts real CPU work per grid cell -- edit this set to
 * widen/narrow what gets detected. */
static std::set<int> ALLOWED_CLASSES = {0, 1, 2, 3, 5, 7};

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
            for (int c = 0; c < OBJ_CLASS_NUM; c++) {
                if (!ALLOWED_CLASSES.empty() && ALLOWED_CLASSES.find(c) == ALLOWED_CLASSES.end()) continue;
                if (score_t[score_offset + c] > score_thres_i8 && score_t[score_offset + c] > max_score) {
                    max_score = score_t[score_offset + c];
                    max_class_id = c;
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
static void downscale_for_inference(unsigned char* src, int src_w, int src_h, unsigned char* dst, int dst_w, int dst_h) {
    for (int y = 0; y < dst_h; y++) {
        int sy = y * src_h / dst_h;
        for (int x = 0; x < dst_w; x++) {
            int sx = x * src_w / dst_w;
            for (int c = 0; c < 3; c++) dst[(y * dst_w + x) * 3 + c] = src[(sy * src_w + sx) * 3 + c];
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
        float bx = d.x / MODEL_SIZE, by = d.y / MODEL_SIZE, bw = d.w / MODEL_SIZE, bh = d.h / MODEL_SIZE;
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
static void on_quit_signal(int) { g_quit = 1; }
static const char* g_web_root = "www";

static struct {
    int streaming;
    int frame;
    double fps, infer_ms, jpeg_ms, read_ms;
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
        "\"frame\":%d,\"fps\":%.2f,\"infer_ms\":%.1f,\"jpeg_ms\":%.1f,\"read_ms\":%.1f,"
        "\"dets\":%d,\"cpu_pct\":%.0f,\"mem_avail_mb\":%d,\"soc_temp_c\":%.1f,"
        "\"display_w\":%d,\"display_h\":%d,\"model_size\":%d,\"source\":\"%s\","
        "\"view\":\"%s\",\"session\":%d,\"session_s\":%.0f,\"objects_total\":%d,"
        "\"has_tilt\":%s,\"pitch\":%.1f,\"roll\":%.1f,\"nav_subscriber\":%s}",
        g_status.streaming ? "true" : "false", g_status.viewer_ip, g_status.viewer_id, g_status.viewers_served,
        g_status.frame, g_status.fps, g_status.infer_ms, g_status.jpeg_ms, g_status.read_ms,
        g_status.dets, g_status.cpu_pct, read_mem_available_mb(), read_soc_temp_c(),
        g_status.display_w, g_status.display_h, MODEL_SIZE, g_status.source ? g_status.source : "",
        g_view_hud ? "hud" : "plain", g_session.session_id,
        g_status.streaming ? session_elapsed_s() : 0.0, g_session.total,
        g_status.has_tilt ? "true" : "false", g_status.pitch, g_status.roll, g_nav_fd >= 0 ? "true" : "false");
    http_send_simple(fd, "200 OK", "application/json", body, (size_t)len);
}

/* Newest first, at most the ring's 200 entries, plus per-class totals. */
static void http_send_objects(int fd) {
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
    int n_live = std::min(g_session.total, SESSION_MAX_OBJECTS);
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
    } else if (!strcmp(target, "/api/objects")) {
        http_send_objects(fd);
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
    while (client_fd < 0 && g_nav_fd < 0 && !g_quit) http_accept_one();
    g_status.streaming = client_fd >= 0;
}

/* libjpeg-turbo encode to an in-memory buffer via the standard
 * jpeg_mem_dest() API (part of libjpeg's public API since v8, real and
 * portable -- not the simplified TurboJPEG API, since jpeglib.h was
 * already the header on hand). jpeg_buf/jpeg_buf_size are kept static
 * and passed back in on every call rather than freed each time --
 * jpeg_mem_dest() reuses the existing allocation if it's already large
 * enough (per its own documented contract) and only reallocs when it
 * needs to grow, avoiding a malloc/free cycle every single frame. */
static unsigned char* jpeg_buf = NULL;
/* jpeg_mem_dest()'s size argument is unsigned long* in libjpeg-turbo (the
 * Ubuntu build) but size_t* in IJG libjpeg 9 (the Buildroot SDK's library). */
#if JPEG_LIB_VERSION >= 90
static size_t jpeg_buf_size = 0;
#else
static unsigned long jpeg_buf_size = 0;
#endif

static unsigned long jpeg_encode_turbo(unsigned char* rgb, int w, int h) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &jpeg_buf, &jpeg_buf_size);

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

    /* jpeg_mem_dest() may have grown jpeg_buf via realloc() during
     * compress -- jpeg_destroy_compress() doesn't touch it, real size
     * actually written is in cinfo.dest's own tracked count, exposed
     * back to us via jpeg_buf_size (jpeg_mem_dest updates *outsize to
     * the real compressed length on jpeg_finish_compress -- confirmed
     * in jpeglib.h's own documented contract for this function). */
    unsigned long real_size = jpeg_buf_size;
    jpeg_destroy_compress(&cinfo);
    return real_size;
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
    if (g_hwjpeg_nv12) rgb_to_nv12(rgb, w, h, dst);
    else memcpy(dst, rgb, (size_t)w * h * 3);
    RK_MPI_SYS_MmzFlushCache(g_hwjpeg_blk, RK_FALSE);

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
    if (live_camera) {
        if (!v4l2_open_and_configure(&v4l2_cap, video_path, display_w, display_h)) {
            printf("RESULT: V4L2_INIT_FAIL\n");
            return 1;
        }
        /* the driver may have negotiated a different real size than
         * requested (stepwise 32x32, 8px steps on this sensor) -- use
         * what it actually reports from here on, not the request */
        display_w = v4l2_cap.width;
        display_h = v4l2_cap.height;
    }
    printf("RESULT: DISPLAY_RES %dx%d (inference stays %dx%d)\n", display_w, display_h, MODEL_SIZE, MODEL_SIZE);
    g_status.display_w = display_w;
    g_status.display_h = display_h;
    g_status.source = video_path;
#ifdef LUCKFOX_HW_JPEG
    hwjpeg_init(display_w, display_h);
#endif

    /* --- RKNN init, zero-copy I/O, ONCE --- */
    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, (void*)model_path, 0, 0, NULL);
    if (ret != RKNN_SUCC) { printf("RESULT: INIT_FAIL ret=%d\n", ret); return 1; }

    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    rknn_tensor_attr input_attr;
    memset(&input_attr, 0, sizeof(input_attr));
    input_attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_NATIVE_INPUT_ATTR, &input_attr, sizeof(input_attr));
    input_attr.type = RKNN_TENSOR_INT8;
    input_attr.fmt = RKNN_TENSOR_NHWC;

    std::vector<rknn_tensor_attr> output_attrs(io_num.n_output);
    memset(output_attrs.data(), 0, sizeof(rknn_tensor_attr) * io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
    }

    rknn_tensor_mem* input_mem = rknn_create_mem(ctx, input_attr.size_with_stride);
    rknn_set_io_mem(ctx, input_mem, &input_attr);
    std::vector<rknn_tensor_mem*> output_mems(io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_mems[i] = rknn_create_mem(ctx, output_attrs[i].size_with_stride);
        rknn_set_io_mem(ctx, output_mems[i], &output_attrs[i]);
    }
    int8_t* in_buf = (int8_t*)input_mem->virt_addr;
    printf("RESULT: MODEL_READY n_output=%d\n", io_num.n_output);

    if (http_start_server(http_port) != 0) return 1;

    AccelState accel;
    accel_discover(&accel);

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
    unsigned char* frame_full = (unsigned char*)malloc(display_w * display_h * 3);
    unsigned char* frame_small = (unsigned char*)malloc(MODEL_SIZE * MODEL_SIZE * 3);
    if (use_rga_downscale) {
        if (drm_init()) {
            g_drm_src = (unsigned char*)drm_alloc_contig(display_w, display_h, 3, &g_src_stride_px);
            g_drm_dst = (unsigned char*)drm_alloc_contig(MODEL_SIZE, MODEL_SIZE, 3, &g_dst_stride_px);
        }
        if (!g_drm_src || !g_drm_dst) {
            printf("RESULT: DRM_ALLOC_FAILED, falling back to CPU downscale\n");
            use_rga_downscale = false;
        }
    }
    float scale_x = (float)display_w / MODEL_SIZE;
    float scale_y = (float)display_h / MODEL_SIZE;
    printf("RESULT: DOWNSCALE_MODE %s\n", use_rga_downscale ? "RGA" : "CPU");

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

        while ((client_fd >= 0 || g_nav_fd >= 0) && !g_quit) {
            struct timespec t_read_start, t_read_end;
            clock_gettime(CLOCK_MONOTONIC, &t_read_start);
            bool got_frame;
            if (live_camera) {
                got_frame = v4l2_capture_frame(&v4l2_cap, frame_full);
            } else {
                size_t n = fread(frame_full, 1, display_w * display_h * 3, ffmpeg_pipe);
                got_frame = (n == (size_t)(display_w * display_h * 3));
            }
            clock_gettime(CLOCK_MONOTONIC, &t_read_end);
            double read_ms = (t_read_end.tv_sec - t_read_start.tv_sec) * 1000.0 +
                              (t_read_end.tv_nsec - t_read_start.tv_nsec) / 1e6;
            if (!got_frame) {
                if (live_camera) printf("RESULT: V4L2_CAPTURE_FAIL\n");
                break; /* video ended, or a real capture error */
            }

            struct timespec t_infer_start, t_infer_end;
            clock_gettime(CLOCK_MONOTONIC, &t_infer_start);

            struct timespec t_ds_start, t_ds_end;
            clock_gettime(CLOCK_MONOTONIC, &t_ds_start);
            if (use_rga_downscale) {
                if (!downscale_for_inference_rga(frame_full, display_w, display_h, frame_small, MODEL_SIZE, MODEL_SIZE)) {
                    fprintf(stderr, "RESULT: RGA_DOWNSCALE_FAILED, falling back to CPU downscale for this and all future frames\n");
                    use_rga_downscale = false;
                    downscale_for_inference(frame_full, display_w, display_h, frame_small, MODEL_SIZE, MODEL_SIZE);
                }
            } else {
                downscale_for_inference(frame_full, display_w, display_h, frame_small, MODEL_SIZE, MODEL_SIZE);
            }
            clock_gettime(CLOCK_MONOTONIC, &t_ds_end);
            double downscale_ms = (t_ds_end.tv_sec - t_ds_start.tv_sec) * 1000.0 +
                                   (t_ds_end.tv_nsec - t_ds_start.tv_nsec) / 1e6;
            for (int i = 0; i < MODEL_SIZE * MODEL_SIZE * 3; i++) in_buf[i] = (int8_t)((int)frame_small[i] - 128);
            ret = rknn_run(ctx, NULL);
            if (ret != RKNN_SUCC) { printf("RESULT: RUN_FAIL ret=%d\n", ret); break; }

            std::vector<Detection> all_dets;
            for (int s = 0; s < NUM_SCALES; s++) {
                rknn_tensor_attr& box_attr = output_attrs[s * 3 + 0];
                rknn_tensor_attr& score_attr = output_attrs[s * 3 + 1];
                rknn_tensor_attr& sum_attr = output_attrs[s * 3 + 2];
                int grid_h = box_attr.dims[1], grid_w = box_attr.dims[2], dfl_len = box_attr.dims[3] / 4;
                decode_scale((int8_t*)output_mems[s * 3 + 0]->virt_addr, box_attr.zp, box_attr.scale,
                             (int8_t*)output_mems[s * 3 + 1]->virt_addr, score_attr.zp, score_attr.scale,
                             (int8_t*)output_mems[s * 3 + 2]->virt_addr, sum_attr.zp, sum_attr.scale,
                             grid_h, grid_w, STRIDES[s], dfl_len, all_dets);
            }
            std::vector<Detection> dets = nms(all_dets);
            clock_gettime(CLOCK_MONOTONIC, &t_infer_end);
            double infer_ms = (t_infer_end.tv_sec - t_infer_start.tv_sec) * 1000.0 +
                               (t_infer_end.tv_nsec - t_infer_start.tv_nsec) / 1e6;

            accel_update(&accel);

            /* --- draw HUD overlay onto the FULL-res frame -- box
             * coordinates rescaled from MODEL_SIZE space up to
             * display_w x display_h --- */
            image_buffer_t img;
            memset(&img, 0, sizeof(img));
            img.width = display_w; img.height = display_h;
            img.width_stride = display_w; img.height_stride = display_h;
            img.format = IMAGE_FORMAT_RGB888;
            img.virt_addr = frame_full;
            img.size = display_w * display_h * 3;

            frame_count++;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (now.tv_sec - fps_t0.tv_sec) + (now.tv_nsec - fps_t0.tv_nsec) / 1e9;
            double fps = elapsed > 0 ? frame_count / elapsed : 0;
            float cpu_pct = read_cpu_percent();
            g_status.frame = frame_count;
            g_status.fps = fps;
            g_status.infer_ms = infer_ms;
            g_status.read_ms = read_ms;
            g_status.dets = (int)dets.size();
            g_status.cpu_pct = cpu_pct;
            std::vector<int> det_ids;
            session_update(dets, &det_ids);

            /* tilt from gravity: roll about the sensor X axis, pitch about Y
             * (standard accelerometer-only formulas; valid while the board
             * isn't accelerating hard -- the EMA smoothing helps there) */
            int has_tilt = accel.found && accel.ema_initialized;
            float tilt_pitch = 0, tilt_roll = 0;
            if (has_tilt) {
                float ax = accel.smoothed_x, ay = accel.smoothed_y, az = accel.smoothed_z;
                tilt_roll = atan2f(ay, az) * 180.0f / (float)M_PI;
                tilt_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
            }
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

            double jpeg_ms = 0, net_ms = 0;
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

            if (http_send_frame(frame_full, display_w, display_h, &jpeg_ms, &net_ms) < 0) {
                printf("RESULT: CLIENT_DISCONNECTED after %d frames\n", frame_count);
                close(client_fd);
                client_fd = -1;
                g_status.streaming = 0;
                session_reset(); /* per-session object log is wiped on disconnect */
            } else {
                g_status.jpeg_ms = jpeg_ms;
            }
            } /* client_fd >= 0: HUD + JPEG only when someone is watching */
            http_poll_requests();

            if (frame_count % 10 == 0) {
                printf("frame=%d fps=%.1f read_ms=%.1f downscale_ms=%.1f infer_ms=%.1f jpeg_ms=%.1f net_ms=%.1f dets=%zu\n",
                       frame_count, fps, read_ms, downscale_ms, infer_ms, jpeg_ms, net_ms, dets.size());
                fflush(stdout);
            }
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
    if (live_camera) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(v4l2_cap.fd, VIDIOC_STREAMOFF, &type);
        for (int i = 0; i < v4l2_cap.n_buffers; i++) munmap(v4l2_cap.buffers[i].start, v4l2_cap.buffers[i].length);
        close(v4l2_cap.fd);
    }
#ifdef LUCKFOX_HW_JPEG
    if (g_hwjpeg_blk) kmsg_mark("main: v4l2 stream closed, Rockit teardown next");
    hwjpeg_shutdown();
#endif
    free(frame_full);
    free(frame_small);
    for (uint32_t i = 0; i < io_num.n_output; i++) rknn_destroy_mem(ctx, output_mems[i]);
    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy(ctx);
#ifdef LUCKFOX_HW_JPEG
    if (g_hwjpeg_nv12) kmsg_mark("main: teardown done, exiting");
#endif
    return 0;
}
