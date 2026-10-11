#!/bin/bash
# Stream-server restart stress test (2026-10-11 freeze investigation).
#
# Each cycle: attach a video viewer and a navigation subscriber, wait a
# random 3-8s, stop the server (SIGTERM, graceful) and let yolo-app-start's
# restart loop bring it back -- the same path a deploy takes. A freeze no
# longer needs a power cycle: the heartbeat health monitor (yolo-health)
# lets the hardware watchdog reset the board; this script waits for that,
# saves the previous boot's kernel log (ramoops) and carries on.
#
#   bash tools/restart_stress.sh [cycles] [with3a|no3a]
#   no3a: stop rkaiq_3A_server first (camera exposure goes wrong, but it
#         tests whether the 3A server is part of the freeze)
set -uo pipefail
CYCLES=${1:-20}
MODE=${2:-with3a}
BOARD=root@172.32.0.70
URL=http://172.32.0.70:8080
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="$ROOT/build/restart-stress-$MODE${TAG:+-$TAG}-$(date +%Y%m%d-%H%M)"
mkdir -p "$LOGDIR"
SSH="ssh -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3 $BOARD"

prepare() {
    # wait until the server is up (after a boot or a reset)
    for i in $(seq 1 60); do curl -s -o /dev/null --max-time 2 "$URL/api/status" && break; sleep 2; done
    # yolo_supervisor owns the hardware watchdog; check it armed this boot
    # (it refuses to after 3 consecutive faults, to avoid a reset loop).
    if ! $SSH 'dmesg | grep -q "yolo-supervisor: armed"'; then
        echo "  WARNING: supervisor not armed this boot -- a hang will need a power cycle" | tee -a "$LOGDIR/cycles.log"
    fi
    if [ "$MODE" = no3a ]; then $SSH 'killall rkaiq_3A_server 2>/dev/null; true'; fi
    # EXTRA_PREP: a command run on the board after every boot/recovery, for
    # A/B experiments (e.g. turning off eMMC runtime PM)
    if [ -n "${EXTRA_PREP:-}" ]; then $SSH "$EXTRA_PREP"; fi
}

prepare
freezes=0; clean=0
for n in $(seq 1 "$CYCLES"); do
    curl -s --max-time 12 "$URL/stream?id=rst$n" -o /dev/null &
    curl -s -N --max-time 12 "$URL/api/detections/stream" -o /dev/null &
    sleep $(( (RANDOM % 6) + 3 ))
    out=$($SSH 'killall luckfox_video_hud_stream_cam luckfox_video_hud_stream_rockit 2>/dev/null; for i in 1 2 3 4 5 6 7 8; do sleep 1; pidof luckfox_video_hud_stream_cam luckfox_video_hud_stream_rockit >/dev/null || break; done; pidof luckfox_video_hud_stream_cam luckfox_video_hud_stream_rockit >/dev/null && echo STUCK || echo exited' 2>&1)
    sleep 8   # restart loop brings it back within ~5s, then model load
    if curl -s -o /dev/null --max-time 3 "$URL/api/status" && [[ "$out" == exited* ]]; then
        clean=$((clean+1)); echo "cycle $n: clean" | tee -a "$LOGDIR/cycles.log"
        continue
    fi
    freezes=$((freezes+1))
    echo "cycle $n: FREEZE/HANG (${out:-no answer}) -- waiting for self-reset" | tee -a "$LOGDIR/cycles.log"
    T0=$(date +%s)
    until ! nc -z -G 1 172.32.0.70 22 2>/dev/null; do sleep 1; [ $(( $(date +%s)-T0 )) -gt 120 ] && break; done
    until nc -z -G 1 172.32.0.70 22 2>/dev/null; do sleep 2; [ $(( $(date +%s)-T0 )) -gt 240 ] && { echo "NO RECOVERY in 4 min -- stopping" | tee -a "$LOGDIR/cycles.log"; exit 1; }; done
    echo "  board back after $(( $(date +%s)-T0 ))s" | tee -a "$LOGDIR/cycles.log"
    sleep 5
    $SSH 'mount -t pstore pstore /sys/fs/pstore 2>/dev/null; cat /sys/fs/pstore/console-ramoops-0 2>/dev/null' > "$LOGDIR/pstore-cycle$n.txt"
    $SSH 'cat /sys/fs/pstore/pmsg-ramoops-0 2>/dev/null' > "$LOGDIR/pmsg-cycle$n.txt"
    echo "  saved crash log: $(wc -l < "$LOGDIR/pstore-cycle$n.txt") lines; supervisor: $(grep FAULT "$LOGDIR/pmsg-cycle$n.txt" | tail -1)" | tee -a "$LOGDIR/cycles.log"
    prepare
done
echo "$MODE: $clean clean, $freezes freezes in $CYCLES cycles" | tee -a "$LOGDIR/cycles.log"
echo "Logs: $LOGDIR"
