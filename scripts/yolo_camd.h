/* Shared between yolo_camd (owns the camera stream) and its client, the
 * stream server. See yolo_camd.c for the protocol. */
#ifndef YOLO_CAMD_H
#define YOLO_CAMD_H
#include <stdint.h>

#define CAMD_SOCKET "\0yolo-camd"
#define CAMD_MAGIC 0x43414d44u /* "CAMD" */
#define CAMD_MAX_BUFFERS 4

typedef struct {
    uint32_t magic;
    uint32_t width, height, stride;
    uint32_t n_buffers;
    uint32_t length[CAMD_MAX_BUFFERS];
} CamdHello;

typedef struct {
    int32_t index;     /* buffer now lent to the client */
    uint32_t sequence; /* V4L2 frame sequence number */
    uint64_t ts_ns;    /* capture timestamp (CLOCK_MONOTONIC) */
} CamdFrame;

#endif
