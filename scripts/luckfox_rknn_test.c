/* Real-hardware RKNN inference check for the Luckfox Pico Ultra
 * (RV1106). Uses the zero-copy DMA memory API (rknn_create_mem +
 * rknn_set_io_mem) -- a REAL, RV1106-specific requirement found
 * 2026-09-08 by comparing a first attempt using the standard
 * rknn_inputs_set()/rknn_outputs_get() API (which rknn_init'd fine but
 * failed at rknn_inputs_set with "context config invalid!", ret=-5)
 * against Rockchip's own yolo11_rv1106_1103.cc reference example, which
 * uses this memory API exclusively and comments "1106 npu only support
 * NHWC in zero copy mode" -- RV1106's NPU memory architecture (tiny
 * DMA-mapped SRAM, not full DRAM access like RK3588-class chips) does
 * not support the plain-buffer path RK3588 demos use.
 *
 * Loads a .rknn model, queries SDK/native input/output attrs, runs one
 * inference pass against a zeroed dummy input (shape only needs to
 * match -- this checks rknn_init/rknn_run complete on REAL hardware,
 * not detection accuracy), and prints the real output tensor shapes.
 * No decode/NMS -- a separate step once this confirms the runtime path
 * itself works.
 *
 * Compiled natively on-device (gcc is present on this Ubuntu 22.04
 * armv7 image) -- no cross-toolchain needed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rknn_api.h"

static void dump_tensor_attr(rknn_tensor_attr* attr) {
    printf("  index=%d name=%s n_dims=%d dims=[%d,%d,%d,%d] n_elems=%d size=%d size_with_stride=%d fmt=%s type=%s qnt=%s zp=%d scale=%f\n",
           attr->index, attr->name, attr->n_dims,
           attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, attr->size_with_stride, get_format_string(attr->fmt),
           get_type_string(attr->type), get_qnt_type_string(attr->qnt_type),
           attr->zp, attr->scale);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s model.rknn\n", argv[0]);
        return 1;
    }
    const char* model_path = argv[1];

    rknn_context ctx = 0;
    printf("=== rknn_init(%s) ===\n", model_path);
    int ret = rknn_init(&ctx, (void*)model_path, 0, 0, NULL);
    if (ret != RKNN_SUCC) {
        printf("RESULT: INIT_FAIL ret=%d\n", ret);
        return 1;
    }
    printf("RESULT: INIT_OK\n");

    rknn_sdk_version ver;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &ver, sizeof(ver));
    if (ret == RKNN_SUCC) {
        printf("SDK: api_version=%s driver_version=%s\n", ver.api_version, ver.drv_version);
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC) {
        printf("RESULT: QUERY_IO_NUM_FAIL ret=%d\n", ret);
        rknn_destroy(ctx);
        return 1;
    }
    printf("io_num: n_input=%d n_output=%d\n", io_num.n_input, io_num.n_output);

    /* RV1106 zero-copy mode: query NATIVE attrs, not the plain
     * RKNN_QUERY_INPUT_ATTR/OUTPUT_ATTR this session's first attempt
     * used. */
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (uint32_t i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_NATIVE_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            printf("RESULT: QUERY_NATIVE_INPUT_ATTR_FAIL index=%d ret=%d\n", i, ret);
            rknn_destroy(ctx);
            return 1;
        }
        printf("native input attr:\n");
        dump_tensor_attr(&input_attrs[i]);
    }

    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            printf("RESULT: QUERY_NATIVE_OUTPUT_ATTR_FAIL index=%d ret=%d\n", i, ret);
            rknn_destroy(ctx);
            return 1;
        }
        printf("native output attr:\n");
        dump_tensor_attr(&output_attrs[i]);
    }

    /* Per Rockchip's own RV1106 reference example: input type UINT8
     * (fuses normalize+quantize onto the NPU), format NHWC (the only
     * format RV1106's NPU supports in zero-copy mode). */
    input_attrs[0].type = RKNN_TENSOR_UINT8;
    input_attrs[0].fmt = RKNN_TENSOR_NHWC;

    rknn_tensor_mem* input_mem = rknn_create_mem(ctx, input_attrs[0].size_with_stride);
    if (input_mem == NULL) {
        printf("RESULT: CREATE_INPUT_MEM_FAIL\n");
        rknn_destroy(ctx);
        return 1;
    }
    memset(input_mem->virt_addr, 0, input_attrs[0].size_with_stride);

    ret = rknn_set_io_mem(ctx, input_mem, &input_attrs[0]);
    if (ret != RKNN_SUCC) {
        printf("RESULT: SET_INPUT_IO_MEM_FAIL ret=%d\n", ret);
        rknn_destroy_mem(ctx, input_mem);
        rknn_destroy(ctx);
        return 1;
    }
    printf("RESULT: INPUT_MEM_SET_OK\n");

    rknn_tensor_mem* output_mems[io_num.n_output];
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_mems[i] = rknn_create_mem(ctx, output_attrs[i].size_with_stride);
        if (output_mems[i] == NULL) {
            printf("RESULT: CREATE_OUTPUT_MEM_FAIL index=%d\n", i);
            rknn_destroy(ctx);
            return 1;
        }
        ret = rknn_set_io_mem(ctx, output_mems[i], &output_attrs[i]);
        if (ret != RKNN_SUCC) {
            printf("RESULT: SET_OUTPUT_IO_MEM_FAIL index=%d ret=%d\n", i, ret);
            rknn_destroy(ctx);
            return 1;
        }
    }
    printf("RESULT: OUTPUT_MEM_SET_OK\n");

    printf("=== rknn_run ===\n");
    ret = rknn_run(ctx, NULL);
    if (ret != RKNN_SUCC) {
        printf("RESULT: RUN_FAIL ret=%d\n", ret);
        rknn_destroy(ctx);
        return 1;
    }

    printf("RESULT: RUN_OK, got %d output(s)\n", io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        printf("  output[%d]: size=%d virt_addr=%p\n", i, output_attrs[i].size, output_mems[i]->virt_addr);
    }

    for (uint32_t i = 0; i < io_num.n_output; i++) {
        rknn_destroy_mem(ctx, output_mems[i]);
    }
    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy(ctx);
    printf("RESULT: REAL_HARDWARE_INFERENCE_PASSED\n");
    return 0;
}
