/* Minimal first-pass real-hardware RKNN inference check for the Luckfox
 * Pico Ultra (RV1106). Loads a .rknn model, queries SDK/input/output
 * attrs, runs one inference pass against a zeroed dummy input (shape
 * only needs to match -- this is checking that rknn_init/rknn_run
 * complete without crashing on REAL hardware, not detection accuracy),
 * and prints the real output tensor shapes. No decode/NMS -- that's a
 * separate step once this confirms the runtime path itself works.
 *
 * Compiled natively on-device (gcc/arm-linux-gnueabihf-gcc is present
 * on this Ubuntu 22.04 armv7 image) -- no cross-toolchain needed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rknn_api.h"

static void dump_tensor_attr(rknn_tensor_attr* attr) {
    printf("  index=%d name=%s n_dims=%d dims=[%d,%d,%d,%d] n_elems=%d size=%d fmt=%s type=%s qnt=%s zp=%d scale=%f\n",
           attr->index, attr->name, attr->n_dims,
           attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt),
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

    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (uint32_t i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            printf("RESULT: QUERY_INPUT_ATTR_FAIL index=%d ret=%d\n", i, ret);
            rknn_destroy(ctx);
            return 1;
        }
        printf("input attr:\n");
        dump_tensor_attr(&input_attrs[i]);
    }

    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            printf("RESULT: QUERY_OUTPUT_ATTR_FAIL index=%d ret=%d\n", i, ret);
            rknn_destroy(ctx);
            return 1;
        }
        printf("output attr:\n");
        dump_tensor_attr(&output_attrs[i]);
    }

    /* Zeroed dummy input, real size from the queried input attr -- this
     * checks the real init/run/get path completes on hardware, not
     * detection correctness (that needs a real decoded image, a later
     * step). */
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    uint32_t in_size = input_attrs[0].size;
    void* in_buf = calloc(1, in_size);
    inputs[0].index = 0;
    inputs[0].type = input_attrs[0].type;
    inputs[0].fmt = input_attrs[0].fmt;
    inputs[0].size = in_size;
    inputs[0].buf = in_buf;

    printf("=== rknn_inputs_set ===\n");
    ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
    if (ret != RKNN_SUCC) {
        printf("RESULT: INPUTS_SET_FAIL ret=%d\n", ret);
        free(in_buf);
        rknn_destroy(ctx);
        return 1;
    }

    printf("=== rknn_run ===\n");
    ret = rknn_run(ctx, NULL);
    if (ret != RKNN_SUCC) {
        printf("RESULT: RUN_FAIL ret=%d\n", ret);
        free(in_buf);
        rknn_destroy(ctx);
        return 1;
    }

    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        outputs[i].want_float = 0;
    }
    printf("=== rknn_outputs_get ===\n");
    ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
    if (ret != RKNN_SUCC) {
        printf("RESULT: OUTPUTS_GET_FAIL ret=%d\n", ret);
        free(in_buf);
        rknn_destroy(ctx);
        return 1;
    }

    printf("RESULT: RUN_OK, got %d output(s)\n", io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        printf("  output[%d]: size=%d buf_ptr=%p\n", i, outputs[i].size, outputs[i].buf);
    }

    rknn_outputs_release(ctx, io_num.n_output, outputs);
    free(in_buf);
    rknn_destroy(ctx);
    printf("RESULT: REAL_HARDWARE_INFERENCE_PASSED\n");
    return 0;
}
