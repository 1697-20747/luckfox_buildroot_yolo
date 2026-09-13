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
 * Then open http://<board-ip>:<port>/ in a browser on the same WiFi.
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
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>
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

/* --- minimal single-client blocking MJPEG-over-HTTP server --- */
static int server_fd = -1;
static int client_fd = -1;

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
    listen(server_fd, 1);
    printf("RESULT: HTTP_SERVER_LISTENING port=%d\n", port);
    return 0;
}

static void http_wait_for_client() {
    printf("Waiting for a browser to connect...\n");
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &len);
    char ipstr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, ipstr, sizeof(ipstr));
    printf("RESULT: CLIENT_CONNECTED %s\n", ipstr);
    /* consume the request line (ignored -- always stream the same feed) */
    char buf[2048];
    recv(client_fd, buf, sizeof(buf), 0);
    const char* header =
        "HTTP/1.0 200 OK\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-cache\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    send(client_fd, header, strlen(header), 0);
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
static unsigned long jpeg_buf_size = 0;

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
static int http_send_frame(unsigned char* rgb, int w, int h, double* out_jpeg_ms, double* out_net_ms) {
    struct timespec t0, t1, t2;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    unsigned long jpeg_size = jpeg_encode_turbo(rgb, w, h);

    clock_gettime(CLOCK_MONOTONIC, &t1);

    char part_header[128];
    int hlen = snprintf(part_header, sizeof(part_header),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n", jpeg_size);
    int ret = 0;
    if (send(client_fd, part_header, hlen, 0) < 0) ret = -1;
    if (ret == 0 && send(client_fd, jpeg_buf, jpeg_size, 0) < 0) ret = -1;
    if (ret == 0 && send(client_fd, "\r\n", 2, 0) < 0) ret = -1;

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

    while (1) {
        http_wait_for_client();

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
        int client_alive = 1;

        while (client_alive) {
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
            {
                HudBounds cluster_box = apex_hud_cluster_bounds(&img);
                apex_hud_draw_horizon_stub(&img, cluster_box);
                apex_hud_draw_roll_ring_stub(&img, cluster_box);
            }
            {
                HudBounds frame_box = apex_hud_frame_bounds(&img);
                apex_hud_draw_compass_tape_stub(&img, frame_box, 0);
                apex_hud_draw_attitude_readouts_stub(&img, frame_box);
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
            frame_count++;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (now.tv_sec - fps_t0.tv_sec) + (now.tv_nsec - fps_t0.tv_nsec) / 1e9;
            double fps = elapsed > 0 ? frame_count / elapsed : 0;
            float cpu_pct = read_cpu_percent();
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

            double jpeg_ms = 0, net_ms = 0;
            if (http_send_frame(frame_full, display_w, display_h, &jpeg_ms, &net_ms) < 0) {
                printf("RESULT: CLIENT_DISCONNECTED after %d frames\n", frame_count);
                client_alive = 0;
            }

            if (frame_count % 10 == 0) {
                printf("frame=%d fps=%.1f read_ms=%.1f downscale_ms=%.1f infer_ms=%.1f jpeg_ms=%.1f net_ms=%.1f dets=%zu\n",
                       frame_count, fps, read_ms, downscale_ms, infer_ms, jpeg_ms, net_ms, dets.size());
                fflush(stdout);
            }
        }

        if (ffmpeg_pipe) pclose(ffmpeg_pipe);
        close(client_fd);
        client_fd = -1;
    }

    if (live_camera) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(v4l2_cap.fd, VIDIOC_STREAMOFF, &type);
        for (int i = 0; i < v4l2_cap.n_buffers; i++) munmap(v4l2_cap.buffers[i].start, v4l2_cap.buffers[i].length);
        close(v4l2_cap.fd);
    }
    free(frame_full);
    free(frame_small);
    for (uint32_t i = 0; i < io_num.n_output; i++) rknn_destroy_mem(ctx, output_mems[i]);
    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy(ctx);
    return 0;
}
