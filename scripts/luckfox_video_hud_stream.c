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
 * Usage: sudo ./luckfox_video_hud_stream <model.rknn> <video-file> <labels.txt> [http-port] [display_w] [display_h]
 * display_w/display_h default to the source video's own resolution
 * (probed via ffprobe) if omitted.
 * Then open http://<board-ip>:<port>/ in a browser on the same WiFi.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <algorithm>
#include <set>

extern "C" {
#include "utils/image_drawing.h"
}

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "rknn_api.h"

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

static size_t jpeg_write_ctx_len = 0;
static void jpeg_write_cb(void* context, void* data, int size) {
    std::vector<unsigned char>* buf = (std::vector<unsigned char>*)context;
    buf->insert(buf->end(), (unsigned char*)data, (unsigned char*)data + size);
}

static int http_send_frame(unsigned char* rgb, int w, int h) {
    std::vector<unsigned char> jpeg_buf;
    stbi_write_jpg_to_func(jpeg_write_cb, &jpeg_buf, w, h, 3, rgb, 50);

    char part_header[128];
    int hlen = snprintf(part_header, sizeof(part_header),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpeg_buf.size());
    if (send(client_fd, part_header, hlen, 0) < 0) return -1;
    if (send(client_fd, jpeg_buf.data(), jpeg_buf.size(), 0) < 0) return -1;
    if (send(client_fd, "\r\n", 2, 0) < 0) return -1;
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s model.rknn video.mp4 [labels.txt] [http-port] [display_w] [display_h]\n", argv[0]);
        return 1;
    }
    const char* model_path = argv[1];
    const char* video_path = argv[2];
    const char* labels_path = argc > 3 ? argv[3] : "coco_80_labels_list.txt";
    int http_port = argc > 4 ? atoi(argv[4]) : HTTP_PORT_DEFAULT;
    setvbuf(stdout, NULL, _IOLBF, 0); /* line-buffer even when redirected to a file/log */
    load_labels(labels_path);

    int display_w = argc > 5 ? atoi(argv[5]) : 0;
    int display_h = argc > 6 ? atoi(argv[6]) : 0;
    if (display_w <= 0 || display_h <= 0) {
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

    unsigned char* frame_full = (unsigned char*)malloc(display_w * display_h * 3);
    unsigned char* frame_small = (unsigned char*)malloc(MODEL_SIZE * MODEL_SIZE * 3);
    float scale_x = (float)display_w / MODEL_SIZE;
    float scale_y = (float)display_h / MODEL_SIZE;

    while (1) {
        http_wait_for_client();

        char ffmpeg_cmd[1024];
        /* Decode/scale to the FULL display resolution -- no -re (don't
         * throttle to source fps, consume as fast as this pipeline
         * can go); fast_bilinear (cheap, real CPU saving on this
         * single-core chip vs the default bicubic). The separate
         * MODEL_SIZE downscale for inference happens in-process below. */
        snprintf(ffmpeg_cmd, sizeof(ffmpeg_cmd),
                 "ffmpeg -i '%s' -loglevel error -vf scale=%d:%d:flags=fast_bilinear -pix_fmt rgb24 -f rawvideo -",
                 video_path, display_w, display_h);
        FILE* ffmpeg_pipe = popen(ffmpeg_cmd, "r");
        if (!ffmpeg_pipe) { printf("RESULT: FFMPEG_SPAWN_FAIL\n"); break; }

        int frame_count = 0;
        struct timespec fps_t0, now;
        clock_gettime(CLOCK_MONOTONIC, &fps_t0);
        int client_alive = 1;

        while (client_alive) {
            struct timespec t_read_start, t_read_end;
            clock_gettime(CLOCK_MONOTONIC, &t_read_start);
            size_t n = fread(frame_full, 1, display_w * display_h * 3, ffmpeg_pipe);
            clock_gettime(CLOCK_MONOTONIC, &t_read_end);
            double read_ms = (t_read_end.tv_sec - t_read_start.tv_sec) * 1000.0 +
                              (t_read_end.tv_nsec - t_read_start.tv_nsec) / 1e6;
            if (n != (size_t)(display_w * display_h * 3)) break; /* video ended */

            struct timespec t_infer_start, t_infer_end;
            clock_gettime(CLOCK_MONOTONIC, &t_infer_start);

            downscale_for_inference(frame_full, display_w, display_h, frame_small, MODEL_SIZE, MODEL_SIZE);
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

            for (auto& d : dets) {
                int dx = (int)(d.x * scale_x), dy = (int)(d.y * scale_y);
                int dw = (int)(d.w * scale_x), dh = (int)(d.h * scale_y);
                unsigned int color = class_colors[d.cls_id];
                draw_rectangle(&img, dx, dy, dw, dh, color, 2);
                char label[96];
                snprintf(label, sizeof(label), "%s %.0f%%", class_names[d.cls_id], d.score * 100);
                fill_rect(frame_full, display_w, display_h, dx, dy - 14, (int)strlen(label) * 8 + 4, 14, 20, 20, 20);
                draw_text(&img, label, dx + 2, dy - 13, COLOR_WHITE, 12);
            }

            /* always-visible HUD readout panel, top-left -- apex_hud's
             * "always draw the instrument, neutral default when idle"
             * convention, loosely */
            fill_rect(frame_full, display_w, display_h, 4, 4, 220, 46, 15, 15, 15);
            char hud_line1[64], hud_line2[64], hud_line3[64];
            frame_count++;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (now.tv_sec - fps_t0.tv_sec) + (now.tv_nsec - fps_t0.tv_nsec) / 1e9;
            double fps = elapsed > 0 ? frame_count / elapsed : 0;
            snprintf(hud_line1, 64, "frame %d  fps %.1f", frame_count, fps);
            snprintf(hud_line2, 64, "infer %.1fms  det %zu", infer_ms, dets.size());
            snprintf(hud_line3, 64, "LUCKFOX RV1106 / yolov8n @ %dx%d disp", display_w, display_h);
            draw_text(&img, hud_line1, 8, 8, COLOR_WHITE, 12);
            draw_text(&img, hud_line2, 8, 22, COLOR_YELLOW, 12);
            draw_text(&img, hud_line3, 8, 36, COLOR_GREEN, 10);

            struct timespec t_send_start, t_send_end;
            clock_gettime(CLOCK_MONOTONIC, &t_send_start);
            if (http_send_frame(frame_full, display_w, display_h) < 0) {
                printf("RESULT: CLIENT_DISCONNECTED after %d frames\n", frame_count);
                client_alive = 0;
            }
            clock_gettime(CLOCK_MONOTONIC, &t_send_end);
            double send_ms = (t_send_end.tv_sec - t_send_start.tv_sec) * 1000.0 +
                              (t_send_end.tv_nsec - t_send_start.tv_nsec) / 1e6;

            if (frame_count % 10 == 0) {
                printf("frame=%d fps=%.1f read_ms=%.1f infer_ms=%.1f send_ms=%.1f dets=%zu\n",
                       frame_count, fps, read_ms, infer_ms, send_ms, dets.size());
                fflush(stdout);
            }
        }

        pclose(ffmpeg_pipe);
        close(client_fd);
        client_fd = -1;
    }

    free(frame_full);
    free(frame_small);
    for (uint32_t i = 0; i < io_num.n_output; i++) rknn_destroy_mem(ctx, output_mems[i]);
    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy(ctx);
    return 0;
}
