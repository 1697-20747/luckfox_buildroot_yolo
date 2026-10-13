/* yolo_supervisor -- owns the hardware watchdog for the whole uptime
 * (luckfox_buildroot_yolo, 2026-10-11).
 *
 * Why: the board's remaining failure mode is a system-level hang in which
 * USB networking, SSH and even a 5-second disk-syncing logger all stop --
 * while nothing reboots it. Feeding the watchdog from the stream server
 * didn't help: the server disarms it on a clean exit, and a restarted
 * server that can't load from a hung disk never re-arms it. Feeding it from
 * a shell loop didn't help either (pidof hung reading /proc).
 *
 * So this small program, started at boot, locks itself into RAM (it never
 * touches the disk again after start-up) and every 2s checks:
 *   1. stream server heartbeat (/tmp/yolo-heartbeat, tmpfs) fresh -- or
 *      absent for less than RESTART_GRACE_S (stops, restarts, deploys);
 *   2. storage alive: a helper thread writes + fsyncs a small file on
 *      /userdata every 10s; the last success must be recent.
 * While both hold it kicks /dev/watchdog. When one fails it records why in
 * the kernel log and in persistent RAM (/dev/pmsg0 -> after the warm reset,
 * /sys/fs/pstore/pmsg-ramoops-0), asks the kernel for SysRq dumps of every
 * task's state and stack (w: blocked, t: all, l: CPU backtrace -- they land
 * in the console log, which ramoops keeps across the warm reset), and stops
 * kicking: the SoC resets ~20s later. Those dumps are the evidence of where
 * the system is actually stuck (kernel-patches/0003 adds SysRq). A periodic status line goes to pmsg too, so the RAM log shows the
 * last healthy state before a failure.
 *
 * Build: tools/build_app.sh. Started by /etc/init.d/S05watchdog.
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/watchdog.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define HEARTBEAT_FILE "/tmp/yolo-heartbeat"
#define IOCHECK_FILE "/userdata/.yolo-iocheck"
#define WATCHDOG_TIMEOUT_S 20
#define HEARTBEAT_STALE_S 20
#define RESTART_GRACE_S 120
#define IO_STALE_S 30
#define CHECK_PERIOD_S 2
/* No endless reset loop: consecutive faults are counted on /userdata; after
 * MAX_CONSECUTIVE_FAULTS the next boot doesn't supervise at all (and says
 * so), leaving the board up for inspection. HEALTHY_RESET_S of good running
 * zeroes the count. */
#define FAULT_COUNT_FILE "/userdata/.yolo-supervisor-faults"
#define MAX_CONSECUTIVE_FAULTS 3
#define HEALTHY_RESET_S 600

static volatile time_t g_io_ok_at = 0;     /* monotonic seconds of last good write+fsync */
static int g_kmsg = -1, g_pmsg = -1, g_sysrq = -1;

static time_t mono_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec;
}

static void note(int to_kmsg, const char* fmt, ...) {
    char msg[256], line[300];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    int n = snprintf(line, sizeof(line), "[%ld] yolo-supervisor: %s\n", (long)mono_s(), msg);
    if (g_pmsg >= 0 && write(g_pmsg, line, n) < 0) { /* best effort */ }
    if (to_kmsg && g_kmsg >= 0) {
        n = snprintf(line, sizeof(line), "<3>yolo-supervisor: %s\n", msg);
        if (write(g_kmsg, line, n) < 0) { /* best effort */ }
    }
}

static int read_fault_count(void) {
    FILE* f = fopen(FAULT_COUNT_FILE, "r");
    int n = 0;
    if (f) { if (fscanf(f, "%d", &n) != 1) n = 0; fclose(f); }
    return n;
}

static void write_fault_count(int n) {
    int fd = open(FAULT_COUNT_FILE, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) return;
    char b[16];
    int len = snprintf(b, sizeof(b), "%d\n", n);
    if (write(fd, b, len) == len) fsync(fd);
    close(fd);
}

/* runs detached: if storage is what hung, this blocks here, not the reset */
static void* record_fault(void* arg) {
    write_fault_count((int)(long)arg);
    return NULL;
}

/* On a storage fault: copy the eMMC controller's debugfs state (registers,
 * bus settings, request state) into the RAM log -- the kernel doesn't print
 * the actual MMC error, and the board resets right after. debugfs is RAM
 * backed, so reading it doesn't touch the hung storage. Files are opened at
 * fault time (procfs/debugfs lookups need no disk). */
static void dump_file_to_pmsg(const char* path) {
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) { note(0, "cannot open %s (errno %d)", path, errno); return; }
    char buf[2048];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return;
    buf[n] = 0;
    note(0, "---- %s ----", path);
    if (g_pmsg >= 0 && write(g_pmsg, buf, n) < 0) { /* best effort */ }
}

/* eMMC controller (dw_mmc @ 0xffa90000) registers, mapped at start-up so a
 * fault-time read needs no lookup. 2026-10-11 capture showed a write with 0
 * of 32 blocks moved, TXDR pending and an empty FIFO -- the internal DMA
 * never fed the FIFO. The IDMAC registers say why (IDSTS fatal-bus-error,
 * current descriptor/buffer address, byte counters). Only meaningful while
 * the controller is clocked (it is, mid-request). */
#define EMMC_REG_BASE 0xffa90000UL
static volatile uint32_t* g_emmc_regs = NULL;
static void map_emmc_regs(void) {
    int fd = open("/dev/mem", O_RDONLY | O_SYNC | O_CLOEXEC);
    if (fd < 0) return;
    void* p = mmap(NULL, 0x1000, PROT_READ, MAP_SHARED, fd, EMMC_REG_BASE);
    close(fd);
    if (p != MAP_FAILED) g_emmc_regs = (volatile uint32_t*)p;
}
static void dump_emmc_regs(void) {
    if (!g_emmc_regs) { note(0, "eMMC registers not mapped"); return; }
    static const struct { unsigned off; const char* name; } named[] = {
        {0x00, "CTRL"}, {0x08, "CLKDIV"}, {0x10, "CLKENA"}, {0x14, "TMOUT"}, {0x18, "CTYPE"},
        {0x1c, "BLKSIZ"}, {0x20, "BYTCNT"}, {0x24, "INTMASK"}, {0x28, "CMDARG"}, {0x2c, "CMD"},
        {0x40, "MINTSTS"}, {0x44, "RINTSTS"}, {0x48, "STATUS"}, {0x4c, "FIFOTH"},
        {0x5c, "TCBCNT"}, {0x60, "TBBCNT"}, {0x64, "DEBNCE"}, {0x80, "BMOD"}, {0x84, "PLDMND"},
        {0x88, "DBADDR"}, {0x8c, "IDSTS"}, {0x90, "IDINTEN"}, {0x94, "DSCADDR"}, {0x98, "BUFADDR"},
    };
    note(0, "---- dw_mmc @ 0x%lx registers ----", EMMC_REG_BASE);
    for (unsigned i = 0; i < sizeof(named) / sizeof(named[0]); i++)
        note(0, "  %-8s (0x%02x) = 0x%08x", named[i].name, named[i].off, g_emmc_regs[named[i].off / 4]);
}

static void* io_checker(void* arg) {
    (void)arg;
    for (;;) {
        int fd = open(IOCHECK_FILE, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
        if (fd >= 0) {
            char b[32];
            int n = snprintf(b, sizeof(b), "%ld\n", (long)mono_s());
            if (write(fd, b, n) == n && fsync(fd) == 0) g_io_ok_at = mono_s();
            close(fd);
        }
        sleep(10);
    }
    return NULL;
}

int main(void) {
    signal(SIGPIPE, SIG_IGN);
    g_kmsg = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
    g_pmsg = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);
    g_sysrq = open("/proc/sysrq-trigger", O_WRONLY | O_CLOEXEC); /* opened now: no lookup needed at fault time */
    map_emmc_regs();

    int faults = read_fault_count();
    if (faults >= MAX_CONSECUTIVE_FAULTS) {
        note(1, "%d consecutive faults recorded in %s: NOT supervising this boot (delete the file to re-enable)",
             faults, FAULT_COUNT_FILE);
        return 0;
    }

    int wdt = open("/dev/watchdog", O_WRONLY | O_CLOEXEC);
    if (wdt < 0) { note(1, "cannot open /dev/watchdog (errno %d), not supervising", errno); return 1; }
    int t = WATCHDOG_TIMEOUT_S;
    ioctl(wdt, WDIOC_SETTIMEOUT, &t);

    g_io_ok_at = mono_s();
    pthread_t io_thread;
    pthread_create(&io_thread, NULL, io_checker, NULL);

    /* after this nothing pages in from disk: a storage hang can't stall us */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) note(1, "mlockall failed (errno %d)", errno);
    note(1, "armed: watchdog %ds, heartbeat stale > %ds, restart grace %ds, storage stale > %ds",
         t, HEARTBEAT_STALE_S, RESTART_GRACE_S, IO_STALE_S);

    time_t hb_missing_since = 0;
    /* Heartbeat age on the MONOTONIC clock: when the file's mtime last
     * changed. The wall clock can jump (no RTC; the web UI sets it from the
     * browser, 2026-10-13): wall - mtime would then look years stale. */
    struct timespec hb_seen_mtime = { 0, 0 };
    time_t hb_changed_at = mono_s();
    time_t last_status = 0;
    time_t started = mono_s();
    int counted_healthy = (faults == 0);
    for (;;) {
        time_t now = mono_s();
        const char* fault = NULL;
        char detail[96] = "";

        struct stat st;
        long hb_age = -1;
        if (stat(HEARTBEAT_FILE, &st) == 0) {
            hb_missing_since = 0;
            if (st.st_mtim.tv_sec != hb_seen_mtime.tv_sec || st.st_mtim.tv_nsec != hb_seen_mtime.tv_nsec) {
                hb_seen_mtime = st.st_mtim;
                hb_changed_at = now;
            }
            hb_age = (long)(now - hb_changed_at);
            if (hb_age > HEARTBEAT_STALE_S) { fault = "stream server heartbeat stale"; snprintf(detail, sizeof(detail), "%lds old", hb_age); }
        } else {
            if (!hb_missing_since) hb_missing_since = now;
            if (now - hb_missing_since > RESTART_GRACE_S) {
                fault = "stream server not running";
                snprintf(detail, sizeof(detail), "no heartbeat for %lds", (long)(now - hb_missing_since));
            }
        }
        long io_age = (long)(now - g_io_ok_at);
        if (!fault && io_age > IO_STALE_S) {
            fault = "storage not responding";
            snprintf(detail, sizeof(detail), "last /userdata write+fsync %lds ago", io_age);
        }

        if (!fault && !counted_healthy && now - started > HEALTHY_RESET_S) {
            write_fault_count(0);
            faults = 0;
            counted_healthy = 1;
            note(0, "healthy for %ds: consecutive fault count reset", HEALTHY_RESET_S);
        }
        if (fault) {
            pthread_t rec;
            if (pthread_create(&rec, NULL, record_fault, (void*)(long)(faults + 1)) == 0) pthread_detach(rec);
            note(1, "FAULT: %s (%s) -- dumping task states, then stopping watchdog kicks (reset in ~%ds)",
                 fault, detail, t);
            if (strstr(fault, "storage")) {
                dump_emmc_regs();
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/regs");
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/ios");
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/state");
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/req");
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/pending_events");
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/completed_events");
                dump_file_to_pmsg("/sys/kernel/debug/mmc0/mmc0:0001/status");
            }
            if (g_sysrq >= 0) {
                /* keep kicking while the dumps print, so they finish before the reset */
                const char* keys = "wtl";
                for (const char* k = keys; *k; k++) {
                    if (write(g_sysrq, k, 1) < 0) note(0, "sysrq '%c' failed (errno %d)", *k, errno);
                    if (write(wdt, "k", 1) < 0) { /* best effort */ }
                    sleep(2);
                }
            } else {
                note(1, "no /proc/sysrq-trigger: no task dump (kernel without SysRq)");
            }
            note(1, "dumps done, stopping watchdog kicks");
            for (;;) pause();   /* no kicks: hardware watchdog resets the SoC */
        }
        if (write(wdt, "k", 1) < 0) { /* keep trying */ }
        if (now - last_status >= 10) {
            note(0, "ok hb_age=%lds io_age=%lds", hb_age, io_age);
            last_status = now;
        }
        sleep(CHECK_PERIOD_S);
    }
}
