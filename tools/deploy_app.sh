#!/bin/bash
# Copy the cross-built stream server (tools/build_app.sh) to the board's
# /root/rknn_test and restart it. /usr/bin/yolo-app-start (buildroot overlay)
# starts it at boot from there.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BOARD=root@172.32.0.70
cd "$ROOT/build/app_src"
# COPYFILE_DISABLE: stop macOS tar adding ._* AppleDouble files
COPYFILE_DISABLE=1 tar -cf - luckfox_video_hud_stream_cam lib www app.env \
    yolov8n-optimized-rv1106-1.6.0.rknn coco_80_labels_list.txt \
  | ssh "$BOARD" 'mkdir -p /root/rknn_test && cd /root/rknn_test && tar -xf - && chown -R root:root . && sync'
# sync BEFORE stopping the old server: 2026-10-11 the board froze right after
# a deploy and the new binary + libjpeg came back as zero-byte files (written
# to the page cache, never flushed). Stopping the server is the risky step.
ssh "$BOARD" 'killall luckfox_video_hud_stream_cam 2>/dev/null; true'
echo "Deployed. If yolo-app-start's restart loop is running it restarts the server within 5s;"
echo "otherwise: ssh $BOARD /usr/bin/yolo-app-start &   (or reboot)"
