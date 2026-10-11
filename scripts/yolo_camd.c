/* yolo_camd: owns the camera stream for the life of the boot.
 *
 * Why (2026-10-11, evidence in docs/vendor-report-emmc-hang-on-camera-restart.md):
 * every eMMC hang on this board followed a camera stream RESTART with rkaiq
 * (3A) active -- the eMMC controller's DMA stalls fetching its first
 * descriptor ~5.5s after the stream comes back on. The first stream start of a
 * boot has never failed. So the stream is started once, here, and never
 * stopped; the YOLO/HUD server can exit and restart freely beside it.
 *
 * Zero-copy hand-off: the capture buffers are exported as DMA-BUFs and passed
 * to the client over a unix socket when it connects (SCM_RIGHTS); the client
 * maps them once. Per frame the client sends 'N' (next) and gets back the
 * index of the newest filled buffer, which stays lent to it -- not requeued --
 * until it sends 'R' (release), sends 'N' again, or disconnects. One client at
 * a time; a new connection replaces the old one.
 *
 *   yolo_camd /dev/video11 960 540
 *
 * Protocol (SOCK_SEQPACKET, abstract socket "\0yolo-camd"):
 *   camd -> client on connect: CamdHello + n_buffers fds
 *   client -> camd: 'N' | 'R'
 *   camd -> client after 'N': CamdFrame
 */
#define _GNU_SOURCE /* accept4 */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "yolo_camd.h"

#define NBUF 4

static int g_vfd = -1;
static int g_dmabuf[NBUF];
static int g_nbuf;

static void kmsg(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void kmsg(const char* fmt, ...) {
    char line[256];
    int n = snprintf(line, sizeof(line), "<6>yolo-camd: ");
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + n, sizeof(line) - n, fmt, ap);
    va_end(ap);
    fprintf(stderr, "%s\n", line + 3);
    int fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
    if (fd >= 0) { (void)!write(fd, line, strlen(line)); close(fd); }
}

static void queue(int idx) {
    struct v4l2_buffer buf;
    struct v4l2_plane plane;
    memset(&buf, 0, sizeof(buf));
    memset(&plane, 0, sizeof(plane));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = idx;
    buf.m.planes = &plane;
    buf.length = 1;
    if (ioctl(g_vfd, VIDIOC_QBUF, &buf) < 0) { kmsg("QBUF %d: %s", idx, strerror(errno)); exit(2); }
}

static int dequeue(uint32_t* seq, uint64_t* ts_ns) {
    struct v4l2_buffer buf;
    struct v4l2_plane plane;
    memset(&buf, 0, sizeof(buf));
    memset(&plane, 0, sizeof(plane));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = &plane;
    buf.length = 1;
    if (ioctl(g_vfd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) return -1;
        kmsg("DQBUF: %s", strerror(errno));
        exit(2);
    }
    *seq = buf.sequence;
    *ts_ns = (uint64_t)buf.timestamp.tv_sec * 1000000000ull + (uint64_t)buf.timestamp.tv_usec * 1000ull;
    return (int)buf.index;
}

static void send_hello(int cfd, const CamdHello* h) {
    struct iovec iov = { (void*)h, sizeof(*h) };
    char ctrl[CMSG_SPACE(sizeof(int) * NBUF)];
    memset(ctrl, 0, sizeof(ctrl));
    struct msghdr msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = CMSG_SPACE(sizeof(int) * g_nbuf);
    struct cmsghdr* c = CMSG_FIRSTHDR(&msg);
    c->cmsg_level = SOL_SOCKET;
    c->cmsg_type = SCM_RIGHTS;
    c->cmsg_len = CMSG_LEN(sizeof(int) * g_nbuf);
    memcpy(CMSG_DATA(c), g_dmabuf, sizeof(int) * g_nbuf);
    if (sendmsg(cfd, &msg, MSG_NOSIGNAL) < 0) kmsg("hello: %s", strerror(errno));
}

int main(int argc, char** argv) {
    const char* dev = argc > 1 ? argv[1] : "/dev/video11";
    int want_w = argc > 2 ? atoi(argv[2]) : 960;
    int want_h = argc > 3 ? atoi(argv[3]) : 540;
    signal(SIGPIPE, SIG_IGN);

    g_vfd = open(dev, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (g_vfd < 0) { kmsg("open %s: %s", dev, strerror(errno)); return 1; }
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = want_w;
    fmt.fmt.pix_mp.height = want_h;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (ioctl(g_vfd, VIDIOC_S_FMT, &fmt) < 0) { kmsg("S_FMT: %s", strerror(errno)); return 1; }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = NBUF;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(g_vfd, VIDIOC_REQBUFS, &req) < 0) { kmsg("REQBUFS: %s", strerror(errno)); return 1; }
    g_nbuf = req.count < NBUF ? (int)req.count : NBUF;
    if (g_nbuf < 3) { kmsg("only %d buffers", g_nbuf); return 1; }

    CamdHello hello;
    memset(&hello, 0, sizeof(hello));
    hello.magic = CAMD_MAGIC;
    hello.width = fmt.fmt.pix_mp.width;
    hello.height = fmt.fmt.pix_mp.height;
    hello.stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    hello.n_buffers = g_nbuf;
    for (int i = 0; i < g_nbuf; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane plane;
        memset(&buf, 0, sizeof(buf));
        memset(&plane, 0, sizeof(plane));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = &plane;
        buf.length = 1;
        if (ioctl(g_vfd, VIDIOC_QUERYBUF, &buf) < 0) { kmsg("QUERYBUF: %s", strerror(errno)); return 1; }
        hello.length[i] = plane.length;
        struct v4l2_exportbuffer exp;
        memset(&exp, 0, sizeof(exp));
        exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        exp.index = i;
        exp.plane = 0;
        exp.flags = O_RDONLY | O_CLOEXEC;
        if (ioctl(g_vfd, VIDIOC_EXPBUF, &exp) < 0) { kmsg("EXPBUF: %s", strerror(errno)); return 1; }
        g_dmabuf[i] = exp.fd;
        queue(i);
    }

    int lfd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, CAMD_SOCKET, sizeof(CAMD_SOCKET) - 1);   /* abstract: leading NUL */
    socklen_t alen = offsetof(struct sockaddr_un, sun_path) + sizeof(CAMD_SOCKET) - 1;
    if (lfd < 0 || bind(lfd, (struct sockaddr*)&addr, alen) < 0 || listen(lfd, 2) < 0) {
        kmsg("socket: %s (another yolo_camd running?)", strerror(errno));
        return 1;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(g_vfd, VIDIOC_STREAMON, &type) < 0) { kmsg("STREAMON: %s", strerror(errno)); return 1; }
    kmsg("streaming %s %ux%u NV12 stride %u, %d buffers; this stream is never stopped",
         dev, hello.width, hello.height, hello.stride, g_nbuf);

    int cfd = -1;          /* the one client */
    int ready = -1;        /* newest filled buffer, not lent */
    int lent = -1;         /* buffer the client is reading */
    bool waiting = false;  /* client asked for a frame before one was ready */
    uint32_t ready_seq = 0; uint64_t ready_ts = 0;
    unsigned long frames = 0;
    time_t last_frame = time(NULL), last_warn = 0;

    for (;;) {
        struct pollfd p[3] = {
            { g_vfd, POLLIN, 0 }, { lfd, POLLIN, 0 }, { cfd, POLLIN, 0 },
        };
        int np = cfd >= 0 ? 3 : 2;
        int r = poll(p, np, 1000);
        if (r < 0 && errno != EINTR) { kmsg("poll: %s", strerror(errno)); return 2; }
        time_t now = time(NULL);
        if (now - last_frame > 5 && now - last_warn > 30) {
            kmsg("no frame for %lds (frames so far %lu)", (long)(now - last_frame), frames);
            last_warn = now;
        }
        if (r <= 0) continue;

        if (p[0].revents & POLLIN) {
            uint32_t seq; uint64_t ts;
            int idx;
            while ((idx = dequeue(&seq, &ts)) >= 0) {
                frames++;
                last_frame = now;
                if (ready >= 0) queue(ready);
                ready = idx; ready_seq = seq; ready_ts = ts;
            }
        }
        if (p[1].revents & POLLIN) {
            int nfd = accept4(lfd, NULL, NULL, SOCK_CLOEXEC);
            if (nfd >= 0) {
                if (cfd >= 0) {
                    kmsg("new client replaces the old one");
                    close(cfd);
                }
                if (lent >= 0) { queue(lent); lent = -1; }
                waiting = false;
                cfd = nfd;
                send_hello(cfd, &hello);
            }
        }
        if (np == 3 && (p[2].revents & (POLLIN | POLLHUP | POLLERR))) {
            char cmd = 0;
            ssize_t n = recv(cfd, &cmd, 1, MSG_DONTWAIT);
            if (n <= 0) {
                if (n < 0 && errno == EAGAIN) continue;
                close(cfd); cfd = -1; waiting = false;
                if (lent >= 0) { queue(lent); lent = -1; }
            } else if (cmd == 'N' || cmd == 'R') {
                if (lent >= 0) { queue(lent); lent = -1; }
                if (cmd == 'N') waiting = true;
            }
        }
        if (waiting && cfd >= 0 && ready >= 0) {
            CamdFrame f = { (int32_t)ready, ready_seq, ready_ts };
            lent = ready; ready = -1; waiting = false;
            if (send(cfd, &f, sizeof(f), MSG_NOSIGNAL) < 0) {
                close(cfd); cfd = -1;
                queue(lent); lent = -1;
            }
        }
    }
}
