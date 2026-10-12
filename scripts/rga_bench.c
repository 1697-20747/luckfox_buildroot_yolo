/* RGA vs CPU benchmark for the stream server's colour conversions
 * (2026-10-12).
 *
 * EVERY buffer handed to the RGA is a DMA-BUF from the CMA heap, imported by
 * fd. Never a plain virtual address: this board's RGA core has no IOMMU
 * ("NONE_MMU[0] cannot support physically discontinuous virtual address"),
 * and the driver's error path for a rejected virtual-address job corrupted
 * the process's mm accounting -- the process's exit then oopsed the kernel
 * (panic + reboot, 2026-10-12, build/rga-crash-20261012). The benchmark also
 * stops at the first RGA error instead of retrying.
 *
 *   rga_bench [iterations]
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <linux/dma-buf.h>

#include "im2d.h"
#include "RgaUtils.h"

#define W 960
#define H 540
#define MW 800
#define MH 448

struct rk_dma_heap_allocation_data { uint64_t len; uint32_t fd; uint32_t fd_flags; uint64_t heap_flags; };
#define RK_DMA_HEAP_IOCTL_ALLOC _IOWR('R', 0x0, struct rk_dma_heap_allocation_data)

typedef struct { int fd; size_t len; unsigned char* va; rga_buffer_handle_t h; } DmaBuf;

static int g_heap = -1;
static int dma_alloc(DmaBuf* b, size_t len, int w, int h, int fmt) {
    struct rk_dma_heap_allocation_data a = { len, 0, O_RDWR | O_CLOEXEC, 0 };
    if (ioctl(g_heap, RK_DMA_HEAP_IOCTL_ALLOC, &a) < 0) { perror("dma heap alloc"); return -1; }
    b->fd = a.fd; b->len = len;
    b->va = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, b->fd, 0);
    if (b->va == MAP_FAILED) { perror("mmap"); return -1; }
    im_handle_param_t p = { (uint32_t)w, (uint32_t)h, (uint32_t)fmt };
    b->h = importbuffer_fd(b->fd, &p);
    if (!b->h) { fprintf(stderr, "importbuffer_fd failed\n"); return -1; }
    return 0;
}
static void cpu_begin(DmaBuf* b, int rw) { struct dma_buf_sync s = { DMA_BUF_SYNC_START | rw }; ioctl(b->fd, DMA_BUF_IOCTL_SYNC, &s); }
static void cpu_end(DmaBuf* b, int rw) { struct dma_buf_sync s = { DMA_BUF_SYNC_END | rw }; ioctl(b->fd, DMA_BUF_IOCTL_SYNC, &s); }

static double now_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }

static void nv12_to_rgb(const unsigned char* nv12, int w, int h, unsigned char* rgb) {
    const unsigned char* uvp = nv12 + (size_t)w * h;
    for (int r = 0; r < h; r++) for (int c = 0; c < w; c++) {
        int y = nv12[r * w + c] - 16, u = uvp[(r / 2) * w + (c / 2) * 2] - 128, v = uvp[(r / 2) * w + (c / 2) * 2 + 1] - 128;
        int R = (298 * y + 409 * v + 128) >> 8, G = (298 * y - 100 * u - 208 * v + 128) >> 8, B = (298 * y + 516 * u + 128) >> 8;
        unsigned char* o = rgb + ((size_t)r * w + c) * 3;
        o[0] = R < 0 ? 0 : R > 255 ? 255 : R; o[1] = G < 0 ? 0 : G > 255 ? 255 : G; o[2] = B < 0 ? 0 : B > 255 ? 255 : B;
    }
}

#define CHECK(st, what) do { if ((st) != IM_STATUS_SUCCESS && (st) != IM_STATUS_NOERROR) { \
    fprintf(stderr, "%s: %s -- stopping\n", what, imStrError(st)); return 2; } } while (0)

int main(int argc, char** argv) {
    int iters = argc > 1 ? atoi(argv[1]) : 50;
    g_heap = open("/dev/rk_dma_heap/rk-dma-heap-cma", O_RDWR | O_CLOEXEC);
    if (g_heap < 0) { perror("open heap"); return 1; }
    DmaBuf nv12, rgb, small, nv12_out;
    if (dma_alloc(&nv12, W * H * 3 / 2, W, H, RK_FORMAT_YCbCr_420_SP) ||
        dma_alloc(&rgb, W * H * 3, W, H, RK_FORMAT_RGB_888) ||
        dma_alloc(&small, MW * MH * 3, MW, MH, RK_FORMAT_RGB_888) ||
        dma_alloc(&nv12_out, W * H * 3 / 2, W, H, RK_FORMAT_YCbCr_420_SP)) return 1;

    cpu_begin(&nv12, DMA_BUF_SYNC_WRITE);
    for (int r = 0; r < H; r++) for (int c = 0; c < W; c++) nv12.va[r * W + c] = 16 + (r + c) % 220;
    for (int r = 0; r < H / 2; r++) for (int c = 0; c < W; c += 2) {
        nv12.va[W * H + r * W + c] = 16 + (c * 7) % 224; nv12.va[W * H + r * W + c + 1] = 16 + (r * 5) % 224;
    }
    cpu_end(&nv12, DMA_BUF_SYNC_WRITE);

    rga_buffer_t src = wrapbuffer_handle(nv12.h, W, H, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dfull = wrapbuffer_handle(rgb.h, W, H, RK_FORMAT_RGB_888);
    rga_buffer_t dsmall = wrapbuffer_handle(small.h, MW, MH, RK_FORMAT_RGB_888);
    rga_buffer_t dnv12 = wrapbuffer_handle(nv12_out.h, W, H, RK_FORMAT_YCbCr_420_SP);

    /* one call of each first: any error stops here before the timed loops */
    IM_STATUS st = imcheck(src, dfull, (im_rect){0}, (im_rect){0});
    CHECK(st, "imcheck NV12->RGB");
    st = imcvtcolor(src, dfull, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT);
    CHECK(st, "NV12->RGB");

    unsigned char* rgb_cpu = malloc(W * H * 3);
    cpu_begin(&nv12, DMA_BUF_SYNC_READ);
    double t0 = now_ms();
    for (int i = 0; i < iters; i++) nv12_to_rgb(nv12.va, W, H, rgb_cpu);
    double cpu_ms = (now_ms() - t0) / iters;
    cpu_end(&nv12, DMA_BUF_SYNC_READ);

    t0 = now_ms();
    for (int i = 0; i < iters; i++) { st = imcvtcolor(src, dfull, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT); CHECK(st, "NV12->RGB"); }
    printf("NV12->RGB 960x540: CPU %.1f ms, RGA %.1f ms\n", cpu_ms, (now_ms() - t0) / iters);

    t0 = now_ms();
    for (int i = 0; i < iters; i++) {
        st = improcess(src, dsmall, (rga_buffer_t){0}, (im_rect){0, 0, W, H}, (im_rect){0, 0, MW, MH}, (im_rect){0}, IM_SYNC | IM_YUV_TO_RGB_BT601_LIMIT);
        CHECK(st, "NV12->RGB 800x448");
    }
    printf("NV12->RGB resized to 800x448 (one RGA op): %.1f ms\n", (now_ms() - t0) / iters);

    t0 = now_ms();
    for (int i = 0; i < iters; i++) { st = imcvtcolor(dfull, dnv12, RK_FORMAT_RGB_888, RK_FORMAT_YCbCr_420_SP, IM_RGB_TO_YUV_BT601_LIMIT); CHECK(st, "RGB->NV12"); }
    printf("RGB->NV12 960x540: RGA %.1f ms\n", (now_ms() - t0) / iters);

    /* cost of making RGA output visible to the CPU (cache invalidate) */
    t0 = now_ms();
    for (int i = 0; i < iters; i++) { cpu_begin(&rgb, DMA_BUF_SYNC_READ); cpu_end(&rgb, DMA_BUF_SYNC_READ); }
    printf("DMA-BUF CPU sync of a 960x540 RGB buffer: %.2f ms\n", (now_ms() - t0) / iters);

    st = imcvtcolor(src, dfull, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT);
    CHECK(st, "NV12->RGB");
    cpu_begin(&rgb, DMA_BUF_SYNC_READ);
    long hist[5] = {0}; int maxd = 0;
    for (int i = 0; i < W * H * 3; i++) {
        int d = abs((int)rgb_cpu[i] - (int)rgb.va[i]); if (d > maxd) maxd = d;
        hist[d == 0 ? 0 : d <= 2 ? 1 : d <= 5 ? 2 : d <= 10 ? 3 : 4]++;
    }
    cpu_end(&rgb, DMA_BUF_SYNC_READ);
    double n = W * H * 3;
    printf("RGA vs CPU NV12->RGB per channel: exact %.1f%%, 1-2 %.1f%%, 3-5 %.1f%%, 6-10 %.1f%%, >10 %.2f%%, max diff %d\n",
           100 * hist[0] / n, 100 * hist[1] / n, 100 * hist[2] / n, 100 * hist[3] / n, 100 * hist[4] / n, maxd);

    releasebuffer_handle(nv12.h); releasebuffer_handle(rgb.h); releasebuffer_handle(small.h); releasebuffer_handle(nv12_out.h);
    return 0;
}
