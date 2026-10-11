#!/bin/bash
# Multi-hour continuous-run soak: one video viewer and one navigation
# subscriber attached the whole time (reconnected every 10 min), and once a
# minute a line with the board's uptime, server fps / timings, memory and
# temperature. A board reset shows up as uptime going backwards (RESET line;
# the previous boot's kernel log and supervisor log are saved).
#
#   bash tools/continuous_soak.sh [hours]
set -uo pipefail
HOURS=${1:-3}
BOARD=root@172.32.0.70
URL=http://172.32.0.70:8080
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="$ROOT/build/continuous-soak-$(date +%Y%m%d-%H%M)"
mkdir -p "$LOGDIR"
SSH="ssh -o ConnectTimeout=5 $BOARD"
END=$(( $(date +%s) + HOURS * 3600 ))
resets=0; last_up=0

( while [ "$(date +%s)" -lt "$END" ]; do
      curl -s --max-time 600 "$URL/stream?id=soak" -o /dev/null; sleep 1
  done ) &
VIEWER=$!
( while [ "$(date +%s)" -lt "$END" ]; do
      curl -s -N --max-time 600 "$URL/api/detections/stream" -o /dev/null; sleep 1
  done ) &
NAV=$!
trap 'kill $VIEWER $NAV 2>/dev/null' EXIT

echo "continuous soak: ${HOURS}h, logs $LOGDIR" | tee -a "$LOGDIR/soak.log"
while [ "$(date +%s)" -lt "$END" ]; do
    sleep 60
    up=$($SSH 'cut -d. -f1 /proc/uptime' 2>/dev/null)
    st=$(curl -s --max-time 3 "$URL/api/status" | python3 -c '
import json,sys
try:
    d=json.load(sys.stdin); print("fps %s read %s infer %s jpeg %s mem %sMB temp %sC streaming %s" % (d["fps"],d["read_ms"],d["infer_ms"],d["jpeg_ms"],d["mem_avail_mb"],d["soc_temp_c"],d["streaming"]))
except Exception: print("status unavailable")')
    if [ -z "$up" ]; then
        echo "$(date +%T) board not answering; $st" | tee -a "$LOGDIR/soak.log"
        continue
    fi
    if [ "$up" -lt "$last_up" ]; then
        resets=$((resets+1))
        echo "$(date +%T) RESET (uptime $last_up -> $up s)" | tee -a "$LOGDIR/soak.log"
        $SSH 'mount -t pstore pstore /sys/fs/pstore 2>/dev/null; cat /sys/fs/pstore/console-ramoops-0' > "$LOGDIR/pstore-reset$resets.txt" 2>/dev/null
        $SSH 'cat /sys/fs/pstore/pmsg-ramoops-0' > "$LOGDIR/pmsg-reset$resets.txt" 2>/dev/null
    fi
    last_up=$up
    echo "$(date +%T) up ${up}s $st" >> "$LOGDIR/soak.log"
done
echo "done: ${HOURS}h, $resets resets" | tee -a "$LOGDIR/soak.log"
