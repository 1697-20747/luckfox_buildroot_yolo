#!/bin/bash
# Hardware-JPEG stop/start stress test (2026-10-11 freeze investigation).
#
# Repeatedly starts the stream server with LUCKFOX_HW_JPEG=1, optionally
# attaches a viewer, stops it with SIGTERM after a varying delay, and checks
# the board survived. Stops at the first freeze, waits for the hardware
# watchdog to reset the board, and copies the previous boot's kernel log
# (ramoops, /sys/fs/pstore) to the Mac. Needs the debug kernel
# (buildroot/kernel-patches/0002) and the S05watchdog feeder, otherwise a
# freeze still needs a power cycle.
#
#   bash tools/hwjpeg_stress.sh [rounds]
set -uo pipefail
ROUNDS=${1:-20}
BOARD=root@172.32.0.70
URL=http://172.32.0.70:8080
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="$ROOT/build/hwjpeg-stress-$(date +%Y%m%d-%H%M)"
mkdir -p "$LOGDIR"
SSH="ssh -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3 $BOARD"

$SSH '[ -e /dev/watchdog ] && pidof watchdog >/dev/null' \
    || { echo "No watchdog feeder running on the board: a freeze would need a power cycle. Stopping."; exit 1; }

# Pause the boot-started server (software mode: safe to stop). Kill the
# restart loop first so it doesn't respawn it; keep rkaiq_3A_server.
$SSH 'killall yolo-app-start 2>/dev/null; killall luckfox_video_hud_stream_cam 2>/dev/null; sleep 2; rm -f /userdata/.yolo-app-starting; sync'

$SSH 'cat /proc/kmsg' > "$LOGDIR/kmsg-live.log" 2>&1 &
KMSG_PID=$!

result="survived all $ROUNDS rounds"
for n in $(seq 1 "$ROUNDS"); do
    viewer=$(( n % 4 != 0 ))          # 3 of every 4 rounds have a viewer
    delay=$(( (RANDOM % 8) + 1 ))     # stop 1-8s after the viewer attaches
    $SSH 'cd /root/rknn_test && (LUCKFOX_HW_JPEG=1 nohup ./luckfox_video_hud_stream_cam yolov8n-optimized-rv1106-1.6.0.rknn /dev/video11 coco_80_labels_list.txt 8080 > /tmp/hw.log 2>&1 < /dev/null &)'
    sleep 7
    [ $viewer = 1 ] && { curl -s --max-time 20 "$URL/stream?id=stress$n" -o /dev/null & }
    sleep "$delay"
    out=$($SSH 'killall -TERM luckfox_video_hud_stream_cam; for i in 1 2 3 4 5 6 7 8 9 10; do sleep 1; pidof luckfox_video_hud_stream_cam >/dev/null || { echo "exited ${i}s"; exit 0; }; done; echo "STILL RUNNING after 10s"' 2>&1)
    rc=$?
    echo "round $n viewer=$viewer stop_after=${delay}s: ${out:-<no answer>}" | tee -a "$LOGDIR/rounds.log"
    if [ $rc -ne 0 ] || ! nc -z -G 3 172.32.0.70 22 2>/dev/null; then
        result="FREEZE in round $n (viewer=$viewer, stop_after=${delay}s)"
        break
    fi
    if [[ "$out" == STILL* ]]; then
        $SSH 'kill -9 $(pidof luckfox_video_hud_stream_cam) 2>/dev/null'
        result="HANG (process stuck, board alive) in round $n"
        $SSH 'cat /tmp/hw.log' > "$LOGDIR/hw-round$n.log" 2>&1
        break
    fi
    sleep 2
done
echo "$result" | tee -a "$LOGDIR/rounds.log"

if [[ "$result" == FREEZE* ]]; then
    echo "Waiting for the watchdog reset and reboot (up to 3 min)..."
    for i in $(seq 1 90); do nc -z -G 2 172.32.0.70 22 2>/dev/null && break; sleep 2; done
    sleep 5
    $SSH 'ls -la /sys/fs/pstore/; for f in /sys/fs/pstore/*; do echo "===== $f"; cat "$f"; done' > "$LOGDIR/pstore.txt" 2>&1
    echo "pstore saved: $LOGDIR/pstore.txt ($(wc -l < "$LOGDIR/pstore.txt") lines)"
fi
kill $KMSG_PID 2>/dev/null
# leave the board serving again, in software mode
$SSH 'pidof luckfox_video_hud_stream_cam >/dev/null || (/usr/bin/yolo-app-start > /dev/null 2>&1 &)' 2>/dev/null
echo "Logs: $LOGDIR"
