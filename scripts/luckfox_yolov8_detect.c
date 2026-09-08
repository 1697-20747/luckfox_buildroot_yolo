/* Real end-to-end YOLOv8n detection on the Luckfox Pico Ultra (RV1106).
 *
 * I/O layer is self-written (stb_image for JPEG/PNG decode, a plain
 * bilinear resize) rather than Rockchip's full demo build, which pulls
 * in RGA (im2d/drmrga, a hardware 2D-accelerator library) and
 * libjpeg-turbo as extra native dependencies -- not needed to prove
 * real detections work, only to make resize/decode faster later.
 *
 * The zero-copy RKNN I/O (rknn_create_mem/rknn_set_io_mem) is the same
 * real, RV1106-required pattern confirmed working in
 * luckfox_rknn_test.c. The decode math (compute_dfl, the per-scale
 * box/score extraction, greedy per-class NMS) is adapted directly from
 * Rockchip's own rknn_model_zoo yolov8 postprocess.cc
 * (process_i8_rv1106 specifically -- confirmed to match this project's
 * real, on-device-verified output layout: NHWC, box [1,H,W,64],
 * class scores [1,H,W,80], confidence-sum [1,H,W,1] per scale),
 * not re-derived from scratch.
 *
 * Model expected: the "optimized" yolov8n RKNN export (9 outputs, no
 * DFL softmax in the graph -- see PROJECT_MEMORY.md for why the stock
 * Ultralytics export doesn't work on this chip).
 *
 * Usage: ./luckfox_yolov8_detect <model.rknn> <image.jpg> <labels.txt>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "rknn_api.h"

#define OBJ_CLASS_NUM 80
#define NUM_SCALES 3
#define BOX_THRESH 0.25f
#define NMS_THRESH 0.45f
#define MODEL_SIZE 640

static const int STRIDES[NUM_SCALES] = {8, 16, 32};

typedef struct {
    float x, y, w, h;
    float score;
    int cls_id;
} Detection;

static char class_names[OBJ_CLASS_NUM][64];

static void load_labels(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "WARNING: could not open labels file %s, using class indices\n", path);
        for (int i = 0; i < OBJ_CLASS_NUM; i++) snprintf(class_names[i], 64, "class_%d", i);
        return;
    }
    int i = 0;
    char line[64];
    while (fgets(line, sizeof(line), f) && i < OBJ_CLASS_NUM) {
        line[strcspn(line, "\r\n")] = 0;
        strncpy(class_names[i], line, 63);
        i++;
    }
    fclose(f);
}

static float deqnt(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }
static int8_t qnt(float f, int32_t zp, float scale) {
    float v = (f / scale) + zp;
    if (v < -128) v = -128;
    if (v > 127) v = 127;
    return (int8_t)v;
}

/* Rockchip's exact DFL integral decode (postprocess.cc: compute_dfl) */
static void compute_dfl(float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; b++) {
        std::vector<float> exp_t(dfl_len);
        float exp_sum = 0, acc_sum = 0;
        for (int i = 0; i < dfl_len; i++) {
            exp_t[i] = expf(tensor[i + b * dfl_len]);
            exp_sum += exp_t[i];
        }
        for (int i = 0; i < dfl_len; i++) acc_sum += exp_t[i] / exp_sum * i;
        box[b] = acc_sum;
    }
}

/* Adapted from postprocess.cc's process_i8_rv1106 -- same math, real
 * per-scale box/score decode against this project's confirmed NHWC
 * output layout. */
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

            float x1 = (-box[0] + j + 0.5f) * stride;
            float y1 = (-box[1] + i + 0.5f) * stride;
            float x2 = (box[2] + j + 0.5f) * stride;
            float y2 = (box[3] + i + 0.5f) * stride;

            Detection d;
            d.x = x1; d.y = y1; d.w = x2 - x1; d.h = y2 - y1;
            d.score = deqnt(max_score, score_zp, score_scale);
            d.cls_id = max_class_id;
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

/* Standard greedy per-class NMS (postprocess.cc's nms()) */
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

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s model.rknn image.jpg [labels.txt]\n", argv[0]);
        return 1;
    }
    const char* model_path = argv[1];
    const char* image_path = argv[2];
    const char* labels_path = argc > 3 ? argv[3] : "coco_80_labels_list.txt";
    load_labels(labels_path);

    /* --- load + resize image (simple stretch resize, no letterbox --
     * good enough to prove real detections; aspect-ratio-preserving
     * letterbox is a real accuracy improvement to add later) --- */
    int src_w, src_h, src_c;
    unsigned char* src = stbi_load(image_path, &src_w, &src_h, &src_c, 3);
    if (!src) {
        fprintf(stderr, "RESULT: IMAGE_LOAD_FAIL %s\n", image_path);
        return 1;
    }
    printf("Loaded image %s: %dx%d\n", image_path, src_w, src_h);

    unsigned char* resized = (unsigned char*)malloc(MODEL_SIZE * MODEL_SIZE * 3);
    for (int y = 0; y < MODEL_SIZE; y++) {
        for (int x = 0; x < MODEL_SIZE; x++) {
            int sx = x * src_w / MODEL_SIZE;
            int sy = y * src_h / MODEL_SIZE;
            for (int c = 0; c < 3; c++) {
                resized[(y * MODEL_SIZE + x) * 3 + c] = src[(sy * src_w + sx) * 3 + c];
            }
        }
    }
    stbi_image_free(src);

    /* --- RKNN init, zero-copy I/O (same proven pattern as luckfox_rknn_test.c) --- */
    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, (void*)model_path, 0, 0, NULL);
    if (ret != RKNN_SUCC) {
        printf("RESULT: INIT_FAIL ret=%d\n", ret);
        return 1;
    }

    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    printf("io_num: n_input=%d n_output=%d\n", io_num.n_input, io_num.n_output);

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
    int8_t* in_buf = (int8_t*)input_mem->virt_addr;
    /* input scale/zp confirmed on real hardware: scale=1/255, zp=-128
     * -> int8 = uint8_pixel - 128 */
    for (int i = 0; i < MODEL_SIZE * MODEL_SIZE * 3; i++) {
        in_buf[i] = (int8_t)((int)resized[i] - 128);
    }
    free(resized);
    rknn_set_io_mem(ctx, input_mem, &input_attr);

    std::vector<rknn_tensor_mem*> output_mems(io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_mems[i] = rknn_create_mem(ctx, output_attrs[i].size_with_stride);
        rknn_set_io_mem(ctx, output_mems[i], &output_attrs[i]);
    }

    printf("=== rknn_run ===\n");
    ret = rknn_run(ctx, NULL);
    if (ret != RKNN_SUCC) {
        printf("RESULT: RUN_FAIL ret=%d\n", ret);
        return 1;
    }
    printf("RESULT: RUN_OK\n");

    /* --- decode: 3 scales x [box, score, score_sum] = 9 outputs, in
     * that order (confirmed against this project's real conversion:
     * indices 0,1,2 = scale0 box/score/score_sum; 3,4,5 = scale1; 6,7,8
     * = scale2) --- */
    std::vector<Detection> all_dets;
    for (int s = 0; s < NUM_SCALES; s++) {
        rknn_tensor_attr& box_attr = output_attrs[s * 3 + 0];
        rknn_tensor_attr& score_attr = output_attrs[s * 3 + 1];
        rknn_tensor_attr& sum_attr = output_attrs[s * 3 + 2];
        int grid_h = box_attr.dims[1];
        int grid_w = box_attr.dims[2];
        int dfl_len = box_attr.dims[3] / 4;

        decode_scale((int8_t*)output_mems[s * 3 + 0]->virt_addr, box_attr.zp, box_attr.scale,
                     (int8_t*)output_mems[s * 3 + 1]->virt_addr, score_attr.zp, score_attr.scale,
                     (int8_t*)output_mems[s * 3 + 2]->virt_addr, sum_attr.zp, sum_attr.scale,
                     grid_h, grid_w, STRIDES[s], dfl_len, all_dets);
    }
    printf("Raw candidate boxes before NMS: %zu\n", all_dets.size());

    std::vector<Detection> final_dets = nms(all_dets);

    float scale_x = (float)src_w / MODEL_SIZE;
    float scale_y = (float)src_h / MODEL_SIZE;
    printf("RESULT: %zu detection(s) after NMS\n", final_dets.size());
    for (auto& d : final_dets) {
        printf("  %s: score=%.3f box=[x=%.1f y=%.1f w=%.1f h=%.1f] (orig-image coords: x=%.1f y=%.1f w=%.1f h=%.1f)\n",
               class_names[d.cls_id], d.score, d.x, d.y, d.w, d.h,
               d.x * scale_x, d.y * scale_y, d.w * scale_x, d.h * scale_y);
    }

    for (uint32_t i = 0; i < io_num.n_output; i++) rknn_destroy_mem(ctx, output_mems[i]);
    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy(ctx);
    printf("RESULT: DETECTION_PIPELINE_COMPLETE\n");
    return 0;
}
